use std::sync::{Arc, Mutex};

use opencv::{
    core::{self, Point, Scalar, Size, Vec3b},
    highgui, imgproc,
    prelude::*,
    videoio,
};

const WINDOW_FRAME: &str = "camera";
const WINDOW_MASK: &str = "mask";
const H_TOL: i32 = 10;
const S_TOL: i32 = 60;
const V_TOL: i32 = 60;
const MIN_AREA: f64 = 150.0;

#[derive(Clone, Copy, Debug)]
struct TrackerParams {
    hsv_target: Vec3b,
}

#[derive(Debug, Default)]
struct UiState {
    click: Option<Point>,
    tracker: Option<TrackerParams>,
}

fn clamp_u8(value: i32) -> u8 {
    value.clamp(0, 255) as u8
}

fn make_mask_from_hsv(hsv: &Mat, target: Vec3b) -> opencv::Result<Mat> {
    let h = target[0] as i32;
    let s = target[1] as i32;
    let v = target[2] as i32;

    let low_s = clamp_u8(s - S_TOL) as f64;
    let high_s = clamp_u8(s + S_TOL) as f64;
    let low_v = clamp_u8(v - V_TOL) as f64;
    let high_v = clamp_u8(v + V_TOL) as f64;

    let mut mask = Mat::default();

    // Teinte OpenCV en HSV: [0, 179]. On gere le wrap-around pres de 0/179.
    if h - H_TOL < 0 {
        let mut mask1 = Mat::default();
        let mut mask2 = Mat::default();

        core::in_range(
            hsv,
            &Scalar::new(0.0, low_s, low_v, 0.0),
            &Scalar::new((h + H_TOL) as f64, high_s, high_v, 0.0),
            &mut mask1,
        )?;
        core::in_range(
            hsv,
            &Scalar::new((180 + (h - H_TOL)) as f64, low_s, low_v, 0.0),
            &Scalar::new(179.0, high_s, high_v, 0.0),
            &mut mask2,
        )?;
        core::bitwise_or(&mask1, &mask2, &mut mask, &core::no_array())?;
    } else if h + H_TOL > 179 {
        let mut mask1 = Mat::default();
        let mut mask2 = Mat::default();

        core::in_range(
            hsv,
            &Scalar::new((h - H_TOL) as f64, low_s, low_v, 0.0),
            &Scalar::new(179.0, high_s, high_v, 0.0),
            &mut mask1,
        )?;
        core::in_range(
            hsv,
            &Scalar::new(0.0, low_s, low_v, 0.0),
            &Scalar::new(((h + H_TOL) - 180) as f64, high_s, high_v, 0.0),
            &mut mask2,
        )?;
        core::bitwise_or(&mask1, &mask2, &mut mask, &core::no_array())?;
    } else {
        core::in_range(
            hsv,
            &Scalar::new((h - H_TOL) as f64, low_s, low_v, 0.0),
            &Scalar::new((h + H_TOL) as f64, high_s, high_v, 0.0),
            &mut mask,
        )?;
    }

    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE,
        Size::new(5, 5),
        Point::new(-1, -1),
    )?;

    let mut opened = Mat::default();
    imgproc::morphology_ex(
        &mask,
        &mut opened,
        imgproc::MORPH_OPEN,
        &kernel,
        Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;

    Ok(opened)
}

fn main() -> opencv::Result<()> {
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    if !videoio::VideoCapture::is_opened(&cam)? {
        return Err(opencv::Error::new(
            core::StsError,
            "Impossible d'ouvrir la camera (index 0).".to_string(),
        ));
    }

    highgui::named_window(WINDOW_FRAME, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(WINDOW_MASK, highgui::WINDOW_AUTOSIZE)?;

    let state = Arc::new(Mutex::new(UiState::default()));
    {
        let state_for_click = Arc::clone(&state);
        highgui::set_mouse_callback(
            WINDOW_FRAME,
            Some(Box::new(move |event, x, y, _flags| {
                if event == highgui::EVENT_LBUTTONDOWN {
                    if let Ok(mut s) = state_for_click.lock() {
                        s.click = Some(Point::new(x, y));
                    }
                }
            })),
        )?;
    }

    println!("Clic gauche dans la fenetre camera pour choisir la couleur cible.");
    println!("Appuyer sur q ou Echap pour quitter.");

    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.empty() {
            continue;
        }

        let mut hsv = Mat::default();
        imgproc::cvt_color(
            &frame,
            &mut hsv,
            imgproc::COLOR_BGR2HSV,
            0,
            core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )?;

        let pending_click = {
            let mut s = state.lock().expect("mutex ui state poisonnee");
            s.click.take()
        };

        if let Some(p) = pending_click {
            if p.x >= 0 && p.y >= 0 && p.x < hsv.cols() && p.y < hsv.rows() {
                let pixel = *hsv.at_2d::<Vec3b>(p.y, p.x)?;
                let mut s = state.lock().expect("mutex ui state poisonnee");
                s.tracker = Some(TrackerParams { hsv_target: pixel });
                println!(
                    "Couleur calibree en ({}, {}): HSV=({}, {}, {})",
                    p.x, p.y, pixel[0], pixel[1], pixel[2]
                );
            }
        }

        let tracker = {
            let s = state.lock().expect("mutex ui state poisonnee");
            s.tracker
        };

        if let Some(params) = tracker {
            let mask = make_mask_from_hsv(&hsv, params.hsv_target)?;

            let moments = imgproc::moments(&mask, true)?;
            if moments.m00 > MIN_AREA {
                let cx = (moments.m10 / moments.m00) as i32;
                let cy = (moments.m01 / moments.m00) as i32;

                imgproc::circle(
                    &mut frame,
                    Point::new(cx, cy),
                    8,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2,
                    imgproc::LINE_8,
                    0,
                )?;

                imgproc::put_text(
                    &mut frame,
                    &format!("Barycentre: ({}, {})", cx, cy),
                    Point::new(10, 30),
                    imgproc::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2,
                    imgproc::LINE_AA,
                    false,
                )?;
            }

            highgui::imshow(WINDOW_MASK, &mask)?;
        } else {
            imgproc::put_text(
                &mut frame,
                "Cliquez sur la couleur cible",
                Point::new(10, 30),
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar::new(0.0, 255.0, 255.0, 0.0),
                2,
                imgproc::LINE_AA,
                false,
            )?;

            let empty = Mat::zeros(frame.rows(), frame.cols(), core::CV_8UC1)?.to_mat()?;
            highgui::imshow(WINDOW_MASK, &empty)?;
        }

        highgui::imshow(WINDOW_FRAME, &frame)?;
        let key = highgui::wait_key(1)?;
        if key == 27 || key == 'q' as i32 {
            break;
        }
    }

    Ok(())
}
