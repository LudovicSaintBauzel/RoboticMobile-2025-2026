// Raspberryopi : cargo build --release --bin encoder_asservpi --no-default-features --features rpi
// Virtual      : cargo build --release --bin encoder_asservpi 
// Run usage : ./target/release/encoder_asservpi [rpm_m1 [rpm_m2]]
use std::env;
use std::io::{self, Write};
use std::sync::{
    atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU64, Ordering},
    Arc,
};
use std::thread;
use std::time::{Duration, Instant};

#[cfg(feature = "rpi")]
use rppal::gpio::{Gpio, InputPin, Level, Trigger};
#[cfg(feature = "rpi")]
use rppal::i2c::I2c;

const ENC1_A: u8 = 19;
const ENC1_B: u8 = 18;
const ENC2_A: u8 = 17;
const ENC2_B: u8 = 16;

const ENCODER_CPR: f64 = 1440.0;
const I2C_ADDR: u16 = 0x0F;
const PID_PERIOD_MS: u64 = 50;

const DEFAULT_RPM: f64 = 80.0;

const I2C_REG_SPEED: u8 = 0x82;
const I2C_REG_DIR: u8 = 0xAA;

const PID_KP: f64 = 0.9;
const PID_KI: f64 = 0.2;
const PID_KD: f64 = 0.02;

const QDEC_TABLE: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

struct Encoder {
    count: AtomicI32,
    last_state: AtomicU8,
    speed_cps_bits: AtomicU64,
    speed_rpm_bits: AtomicU64,
}

impl Encoder {
    fn new() -> Self {
        Self {
            count: AtomicI32::new(0),
            last_state: AtomicU8::new(0),
            speed_cps_bits: AtomicU64::new(0),
            speed_rpm_bits: AtomicU64::new(0),
        }
    }

    fn speed_cps(&self) -> f64 {
        f64::from_bits(self.speed_cps_bits.load(Ordering::Relaxed))
    }

    fn speed_rpm(&self) -> f64 {
        f64::from_bits(self.speed_rpm_bits.load(Ordering::Relaxed))
    }

    fn set_speed(&self, cps: f64, rpm: f64) {
        self.speed_cps_bits.store(cps.to_bits(), Ordering::Relaxed);
        self.speed_rpm_bits.store(rpm.to_bits(), Ordering::Relaxed);
    }
}

// Wrapper pour rester compatible avec des cibles ou AtomicU64 peut etre plus limite.
// Sur Raspberry Pi 64-bit, AtomicU64 est nativement supporte.
//struct AtomicU64Compat(std::sync::atomic::AtomicU64);

// impl AtomicU64Compat {
//     fn new(value: u64) -> Self {
//         Self(std::sync::atomic::AtomicU64::new(value))
//     }

//     fn load(&self, order: Ordering) -> u64 {
//         self.0.load(order)
//     }

//     fn store(&self, value: u64, order: Ordering) {
//         self.0.store(value, order)
//     }
// }

#[cfg(feature = "rpi")]
struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: f64,
    out_min: f64,
    out_max: f64,
}

#[cfg(feature = "rpi")]
impl PidController {
    fn new(kp: f64, ki: f64, kd: f64, out_min: f64, out_max: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            out_min,
            out_max,
        }
    }

    fn update(&mut self, setpoint: f64, measured: f64, dt: f64) -> f64 {
        let error = setpoint - measured;
        self.integral += error * dt;
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };

        let mut output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        if output > self.out_max {
            output = self.out_max;
        } else if output < self.out_min {
            output = self.out_min;
        }

        // Anti-windup simple: on borne aussi l'integrale.
        if self.ki != 0.0 {
            let i_max = self.out_max / self.ki.abs();
            let i_min = -i_max;
            if self.integral > i_max {
                self.integral = i_max;
            } else if self.integral < i_min {
                self.integral = i_min;
            }
        }

        self.prev_error = error;
        output
    }
}

#[cfg(feature = "rpi")]
fn saturate_to_u8(value: f64) -> u8 {
    value.clamp(0.0, 255.0).round() as u8
}

#[cfg(feature = "rpi")]
fn direction_byte(rpm1: f64, rpm2: f64) -> u8 {
    let d1: u8 = if rpm1 >= 0.0 { 0x01 } else { 0x02 };
    let d2: u8 = if rpm2 >= 0.0 { 0x01 } else { 0x02 };
    d1 | (d2 << 2)
}

fn update_encoder_from_state(enc: &Encoder, new_state: u8) {
    let last_state = enc.last_state.load(Ordering::Relaxed);
    let idx = ((last_state << 2) | new_state) as usize;

    enc.count
        .fetch_add(QDEC_TABLE[idx] as i32, Ordering::Relaxed);
    enc.last_state.store(new_state, Ordering::Relaxed);
}

#[cfg(feature = "rpi")]
fn read_state(pin_a: &InputPin, pin_b: &InputPin) -> u8 {
    let a = if pin_a.read() == Level::High { 1 } else { 0 };
    let b = if pin_b.read() == Level::High { 1 } else { 0 };
    (a << 1) | b
}

#[cfg(feature = "rpi")]
fn update_encoder(enc: &Encoder, pin_a: &InputPin, pin_b: &InputPin) {
    let new_state = read_state(pin_a, pin_b);
    update_encoder_from_state(enc, new_state);
}

#[cfg(feature = "rpi")]
fn configure_input_pin(gpio: &Gpio, bcm_pin: u8) -> Result<InputPin, rppal::gpio::Error> {
    let mut pin = gpio.get(bcm_pin)?.into_input_pullup();
    pin.set_interrupt(Trigger::Both, None)?;
    Ok(pin)
}

fn spawn_speed_thread(
    running: Arc<AtomicBool>,
    enc1: Arc<Encoder>,
    enc2: Arc<Encoder>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut prev_count_1 = enc1.count.load(Ordering::Relaxed);
        let mut prev_count_2 = enc2.count.load(Ordering::Relaxed);
        let mut t_prev = Instant::now();

        while running.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(100));

            let now_count_1 = enc1.count.load(Ordering::Relaxed);
            let now_count_2 = enc2.count.load(Ordering::Relaxed);
            let t_now = Instant::now();

            let dt = (t_now - t_prev).as_secs_f64();
            let dc_1 = now_count_1 - prev_count_1;
            let dc_2 = now_count_2 - prev_count_2;

            if dt > 0.0 {
                let cps1 = dc_1 as f64 / dt;
                let rpm1 = (cps1 / ENCODER_CPR) * 60.0;
                enc1.set_speed(cps1, rpm1);

                let cps2 = dc_2 as f64 / dt;
                let rpm2 = (cps2 / ENCODER_CPR) * 60.0;
                enc2.set_speed(cps2, rpm2);
            }

            prev_count_1 = now_count_1;
            prev_count_2 = now_count_2;
            t_prev = t_now;
        }
    })
}

#[cfg(feature = "rpi")]
fn spawn_pid_i2c_thread(
    running: Arc<AtomicBool>,
    enc1: Arc<Encoder>,
    enc2: Arc<Encoder>,
    target_rpm1: f64,
    target_rpm2: f64,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut i2c = match I2c::new() {
            Ok(bus) => bus,
            Err(err) => {
                eprintln!("Erreur I2C: ouverture du bus impossible: {err}");
                return;
            }
        };

        if let Err(err) = i2c.set_slave_address(I2C_ADDR) {
            eprintln!("Erreur I2C: impossible de selectionner l'adresse 0x{I2C_ADDR:02X}: {err}");
            return;
        }

        // Direction initiale
        let mut last_dir = direction_byte(target_rpm1, target_rpm2);
        if let Err(err) = i2c.write(&[I2C_REG_DIR, last_dir, 0x01]) {
            eprintln!("Erreur I2C write reg 0xAA (direction initiale): {err}");
        }

        let mut pid1 = PidController::new(PID_KP, PID_KI, PID_KD, 0.0, 255.0);
        let mut pid2 = PidController::new(PID_KP, PID_KI, PID_KD, 0.0, 255.0);
        let mut t_prev = Instant::now();

        while running.load(Ordering::Relaxed) {
            let t_now = Instant::now();
            let dt = (t_now - t_prev).as_secs_f64();
            t_prev = t_now;

            let rpm1 = enc1.speed_rpm().abs();
            let rpm2 = enc2.speed_rpm().abs();

            let cmd1 = saturate_to_u8(pid1.update(target_rpm1.abs(), rpm1, dt));
            let cmd2 = saturate_to_u8(pid2.update(target_rpm2.abs(), rpm2, dt));

            // Mettre a jour la direction si elle a change
            let cur_dir = direction_byte(target_rpm1, target_rpm2);
            if cur_dir != last_dir {
                if let Err(err) = i2c.write(&[I2C_REG_DIR, cur_dir, 0x01]) {
                    eprintln!("Erreur I2C write reg 0xAA (direction): {err}");
                }
                last_dir = cur_dir;
            }

            if let Err(err) = i2c.write(&[I2C_REG_SPEED, cmd1, cmd2]) {
                eprintln!("Erreur I2C write reg 0x82: {err}");
            }

            thread::sleep(Duration::from_millis(PID_PERIOD_MS));
        }

        let _ = i2c.write(&[I2C_REG_SPEED, 0x00, 0x00]);
    })
}

fn print_status(enc1: &Encoder, enc2: &Encoder) -> Result<(), io::Error> {
    let c1 = enc1.count.load(Ordering::Relaxed);
    let c2 = enc2.count.load(Ordering::Relaxed);
    let cps1 = enc1.speed_cps();
    let rpm1 = enc1.speed_rpm();
    let cps2 = enc2.speed_cps();
    let rpm2 = enc2.speed_rpm();

    print!(
        "\rENC1 pos={} | v={:.2} counts/s | {:.2} RPM || ENC2 pos={} | v={:.2} counts/s | {:.2} RPM",
        c1, cps1, rpm1, c2, cps2, rpm2
    );
    io::stdout().flush()
}

#[cfg(feature = "rpi")]
fn run_rpi(
    running: Arc<AtomicBool>,
    enc1: Arc<Encoder>,
    enc2: Arc<Encoder>,
    target_rpm1: f64,
    target_rpm2: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let gpio = Gpio::new()?;

    let enc1_a = configure_input_pin(&gpio, ENC1_A)?;
    let enc1_b = configure_input_pin(&gpio, ENC1_B)?;
    let enc2_a = configure_input_pin(&gpio, ENC2_A)?;
    let enc2_b = configure_input_pin(&gpio, ENC2_B)?;

    enc1.last_state
        .store(read_state(&enc1_a, &enc1_b), Ordering::Relaxed);
    enc2.last_state
        .store(read_state(&enc2_a, &enc2_b), Ordering::Relaxed);

    let speed_thread =
        spawn_speed_thread(Arc::clone(&running), Arc::clone(&enc1), Arc::clone(&enc2));
    let i2c_thread = spawn_pid_i2c_thread(
        Arc::clone(&running),
        Arc::clone(&enc1),
        Arc::clone(&enc2),
        target_rpm1,
        target_rpm2,
    );

    println!(
        "Comptage + vitesse (10 Hz) avec PID I2C. Consignes: M1={target_rpm1:.1} RPM, M2={target_rpm2:.1} RPM. Ctrl+C pour arreter."
    );

    // Boucle d'evenements: on attend les fronts sur les 4 voies simultanement.
    let pins: &[&InputPin] = &[&enc1_a, &enc1_b, &enc2_a, &enc2_b];

    while running.load(Ordering::Relaxed) {
        match gpio.poll_interrupts(pins, false, Some(Duration::from_millis(10))) {
            Ok(Some(_)) => {
                update_encoder(&enc1, &enc1_a, &enc1_b);
                update_encoder(&enc2, &enc2_a, &enc2_b);
            }
            Ok(None) => {}
            Err(_) => break, // Interruption (signal) => sortir proprement
        }

        let _ = print_status(&enc1, &enc2);
    }

    running.store(false, Ordering::Relaxed);
    let _ = speed_thread.join();
    let _ = i2c_thread.join();

    // Securite: envoyer zero une derniere fois apres le join du thread PID.
    {
        if let Ok(mut i2c) = I2c::new() {
            let _ = i2c.set_slave_address(I2C_ADDR);
            let _ = i2c.write(&[I2C_REG_SPEED, 0x00, 0x00]);
        }
    }

    Ok(())
}

#[cfg(not(feature = "rpi"))]
fn run_virtual(
    running: Arc<AtomicBool>,
    enc1: Arc<Encoder>,
    enc2: Arc<Encoder>,
) -> Result<(), Box<dyn std::error::Error>> {
    let speed_thread =
        spawn_speed_thread(Arc::clone(&running), Arc::clone(&enc1), Arc::clone(&enc2));

    println!("Mode virtuel actif (pas de GPIO Raspberry Pi). Ctrl+C pour arreter.");

    let seq_forward: [u8; 4] = [0b00, 0b01, 0b11, 0b10];
    let seq_reverse: [u8; 4] = [0b00, 0b10, 0b11, 0b01];
    let mut i1 = 0usize;
    let mut i2 = 0usize;

    while running.load(Ordering::Relaxed) {
        i1 = (i1 + 1) % seq_forward.len();
        i2 = (i2 + 1) % seq_reverse.len();

        update_encoder_from_state(&enc1, seq_forward[i1]);
        update_encoder_from_state(&enc2, seq_reverse[i2]);

        print_status(&enc1, &enc2)?;
        thread::sleep(Duration::from_millis(20));
    }

    running.store(false, Ordering::Relaxed);
    let _ = speed_thread.join();
    Ok(())
}

fn parse_targets() -> Result<(f64, f64), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() > 3 {
        eprintln!("Usage : {} [rpm_m1 [rpm_m2]]", args[0]);
        eprintln!("  RPM positif = avant (CW), RPM negatif = arriere (CCW)");
        std::process::exit(1);
    }
    let mut rpm1 = DEFAULT_RPM;
    let mut rpm2 = DEFAULT_RPM;
    if args.len() >= 2 {
        rpm1 = args[1].parse::<f64>().map_err(|_| format!("Erreur argument rpm_m1 : {}", args[1]))?;
        rpm2 = rpm1;
    }
    if args.len() == 3 {
        rpm2 = args[2].parse::<f64>().map_err(|_| format!("Erreur argument rpm_m2 : {}", args[2]))?;
    }
    Ok((rpm1, rpm2))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (target_rpm1, target_rpm2) = parse_targets()?;

    let running = Arc::new(AtomicBool::new(true));
    {
        let running_for_signal = Arc::clone(&running);
        ctrlc::set_handler(move || {
            running_for_signal.store(false, Ordering::Relaxed);
        })?;
    }

    let enc1 = Arc::new(Encoder::new());
    let enc2 = Arc::new(Encoder::new());

    #[cfg(feature = "rpi")]
    run_rpi(Arc::clone(&running), Arc::clone(&enc1), Arc::clone(&enc2), target_rpm1, target_rpm2)?;

    #[cfg(not(feature = "rpi"))]
    run_virtual(Arc::clone(&running), Arc::clone(&enc1), Arc::clone(&enc2))?;

    println!(
        "\nFinal ENC1={} | ENC2={}",
        enc1.count.load(Ordering::Relaxed),
        enc2.count.load(Ordering::Relaxed)
    );

    Ok(())
}
