use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use esp_radio::ieee802154::{Config, Frame, Ieee802154, ReceivedFrame};
use ieee802154::mac::{
    Address, FrameContent, FrameType, FrameVersion, Header, PanId, ShortAddress,
};

static SHUTDOWN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static RADIO_DROPPED_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub fn trigger_shutdown() {
    SHUTDOWN_SIGNAL.signal(());
}

pub async fn wait_radio_dropped() {
    RADIO_DROPPED_SIGNAL.wait().await;
}

#[embassy_executor::task]
pub async fn run(ieee802154: Ieee802154<'static>, spawner: Spawner) {
    spawner.spawn(ieee802154_run(ieee802154).unwrap());

    let send_ticker_duration = Duration::from_millis(10);
    let mut send_ticker = Ticker::every(send_ticker_duration);

    let txmessage_pub = IEEE802154_SEND.sender();
    let rxmessage_sub = IEEE802154_RECEIVE.receiver();

    loop {
        send_ticker.next().await;
        let mut buffer = heapless::Vec::new();
        buffer.extend_from_slice(b"Hello, world!").unwrap();
        txmessage_pub.send(buffer).await;
    }
}

static IEEE802154_SEND: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 64>, 64> =
    Channel::new();
static IEEE802154_RECEIVE: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 64>, 64> =
    Channel::new();

#[embassy_executor::task]
async fn ieee802154_run(mut ieee802154: Ieee802154<'static>) {
    use embassy_futures::select::{Either3::*, select3};
    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: true,
        pan_id: Some(0x4242),
        short_addr: Some(0x2222),
        cca_mode: esp_radio::ieee802154::CcaMode::Carrier,
        txpower: 20,
        rx_when_idle: true,
        auto_ack_tx: false,
        auto_ack_rx: false,
        ..Default::default()
    });

    let local_timeout = Duration::from_millis(50);
    let remote_timeout = Duration::from_secs(4);

    let mut ieee802154 = AsyncIeee802154::new(ieee802154);

    let ieee802154_send_sub = IEEE802154_SEND.receiver();
    let ieee802154_receive_pub = IEEE802154_RECEIVE.sender();

    loop {
        match select3(
            ieee802154.receive(),
            ieee802154_send_sub.receive(),
            SHUTDOWN_SIGNAL.wait(),
        )
        .await
        {
            First(rxmessage) => {
                info!("Received message: {:x}", rxmessage);
                if ieee802154_receive_pub.is_full() {
                    error!("ieee802154_receive_pub is full");
                    ieee802154_receive_pub.clear();
                }
                ieee802154_receive_pub.send(rxmessage).await;
            }
            Second(txmessage) => {
                if let Err(err) = ieee802154
                    .transmit_txmessage(txmessage, 5, remote_timeout)
                    .await
                {}
            }
            Third(_) => {
                warn!("ieee802154 shutdown signal received, dropping radio...");
                break;
            }
        }
    }
    // ieee802154 (containing Ieee802154) is dropped here.
    // This triggers PhyClockGuard::drop() -> enable_phy(false) -> clk_i2c_mst_en = false
    // But the MAC hardware was never sent Command::Stop, so it may still be active.
    drop(ieee802154);
    warn!("radio dropped, signaling main");
    RADIO_DROPPED_SIGNAL.signal(());
}

#[derive(Debug)]
pub enum AsyncIeee802154Error {
    Timeout,
    Ieee802154(esp_radio::ieee802154::Error),
    SerdeEncrypt,
}

impl defmt::Format for AsyncIeee802154Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Timeout => defmt::write!(f, "Timeout",),
            Self::Ieee802154(err) => defmt::write!(f, "Ieee802154({:?})", defmt::Debug2Format(err)),
            Self::SerdeEncrypt => defmt::write!(f, "SerdeEncrypt",),
        }
    }
}

impl From<esp_radio::ieee802154::Error> for AsyncIeee802154Error {
    fn from(err: esp_radio::ieee802154::Error) -> Self {
        Self::Ieee802154(err)
    }
}

pub struct AsyncIeee802154 {
    ieee802154: Ieee802154<'static>,
    tx_done_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    rx_available_signal: &'static Signal<CriticalSectionRawMutex, ()>,

    rxmessage_buffer: heapless::Vec<heapless::Vec<u8, 64>, 16>,

    tx_seq_number: u8,
    rx_seq_number: u8,
}

impl AsyncIeee802154 {
    pub fn new(mut ieee802154: Ieee802154<'static>) -> Self {
        static TX_DONE_SIGNAL: embassy_sync::signal::Signal<
            embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
            (),
        > = embassy_sync::signal::Signal::new();
        static RX_AVAILABLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

        ieee802154.set_rx_available_callback_fn(|| {
            RX_AVAILABLE_SIGNAL.signal(());
        });
        ieee802154.set_tx_done_callback_fn(|| {
            TX_DONE_SIGNAL.signal(());
        });
        ieee802154.start_receive();

        Self {
            ieee802154,
            tx_done_signal: &TX_DONE_SIGNAL,
            rx_available_signal: &RX_AVAILABLE_SIGNAL,
            tx_seq_number: 0,
            rx_seq_number: 0,
            rxmessage_buffer: heapless::Vec::new(),
        }
    }

    pub async fn transmit_txmessage(
        &mut self,
        txmessage: heapless::Vec<u8, 64>,
        retry: u8,
        timeout: Duration,
    ) -> Result<(), AsyncIeee802154Error> {
        let elapsed = Instant::now();
        let buffer = txmessage;
        for current_retry in 0..retry {
            let chunks = buffer.chunks(100);
            let chunks_count = chunks.len();
            for (c, chunk) in chunks.enumerate() {
                let frame = Frame {
                    header: Header {
                        frame_type: FrameType::Data,
                        frame_pending: c == 0,
                        ack_request: true,
                        pan_id_compress: false,
                        seq_no_suppress: false,
                        ie_present: false,
                        version: FrameVersion::Ieee802154_2003,
                        seq: self.tx_seq_number,
                        destination: Some(Address::Short(
                            PanId(chunks_count as u16),
                            ShortAddress(c as u16),
                        )),
                        source: Some(Address::Short(PanId(0x2222), ShortAddress(0x2222))),
                        auxiliary_security_header: None,
                    },
                    content: FrameContent::Data,
                    payload: chunk.to_vec(),
                    footer: [0, 0],
                };
                if self
                    .transmit_raw(&frame, Duration::from_millis(100))
                    .await
                    .is_err()
                {
                    error!("transmit_raw failed");
                }

                self.tx_seq_number = self.tx_seq_number.wrapping_add(1);
            }
        }

        Ok(())
    }

    async fn transmit_raw(
        &mut self,
        frame: &Frame,
        timeout: Duration,
    ) -> Result<(), AsyncIeee802154Error> {
        self.tx_done_signal.reset();
        //self.rx_available_signal.reset();
        self.ieee802154.transmit(frame, false)?;
        if with_timeout(timeout, self.tx_done_signal.wait())
            .await
            .is_err()
        {
            warn!(
                "timeout waiting for tx_done_signal, timeout was: {}ms",
                timeout.as_millis()
            );
        }

        Ok(())
    }

    fn frame_seq_number_check(&mut self, frame: &ReceivedFrame) -> bool {
        let new_rx_seq_number = frame.frame.header.seq;
        if new_rx_seq_number == self.rx_seq_number {
            warn!("frame with same seq number received");
            false
        } else if new_rx_seq_number == self.rx_seq_number.wrapping_add(1) {
            self.rx_seq_number = new_rx_seq_number;
            true
        } else if new_rx_seq_number == 0 && self.rx_seq_number == 0 {
            warn!("frame seq number both 0");
            true
        } else {
            warn!(
                "frame seq number out of order, expected: {}, got: {}",
                self.rx_seq_number, new_rx_seq_number
            );
            self.rx_seq_number = new_rx_seq_number;
            true
        }
    }

    pub async fn receive_raw(&mut self) -> ReceivedFrame {
        let msg = self.ieee802154.received();
        if let Some(Ok(frame)) = msg {
            if self.frame_seq_number_check(&frame) {
                warn!("early frame return");
                return frame;
            }
        }
        loop {
            self.rx_available_signal.wait().await;

            if let Some(Ok(frame)) = self.ieee802154.received() {
                if self.frame_seq_number_check(&frame) {
                    return frame;
                }
            }
        }
    }

    async fn internal_receive(&mut self) -> heapless::Vec<u8, 64> {
        loop {
            let received_frame = self.receive_raw().await;
            if received_frame.frame.payload.len() < 2 {
                error!("received_frame.frame.payload.len() < 2");
            } else {
                let frame =
                    &received_frame.frame.payload[0..received_frame.frame.payload.len() - 2];
                let mut rxmessage = heapless::Vec::new();
                rxmessage.extend_from_slice(frame).unwrap();
                return rxmessage;
            }
        }
    }

    pub async fn receive(&mut self) -> heapless::Vec<u8, 64> {
        loop {
            if let Some(rxmessage) = self.rxmessage_buffer.pop() {
                warn!("message from buffer");
                return rxmessage;
            }
            return self.internal_receive().await;
        }
    }
}
