use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Context, Node};
use std::sync::{Arc, Mutex};
//use std::time::Duration;
use futures::stream::StreamExt;
use futures::future;
use cgmath::{Deg, Rad, Euler, Quaternion};

#[derive(Clone, Default)]
struct State {
    // markers 0 and 1 define the buffer position
    marker_0: Option<TransformStamped>,
    marker_1: Option<TransformStamped>,
    table_height: Option<f64>,

    // computed results
    buffer_transform: Option<TransformStamped>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ros_ctx = Context::create()?;
    let mut node = Node::create(ros_ctx, "buffer_position_estimator", "")?;

    let tf_sub = node.subscribe::<TFMessage>("/tf", r2r::QosProfile::default())?;
    let tf_pub = node.create_publisher::<TFMessage>("/tf", r2r::QosProfile::default())?;

    let state = Arc::new(Mutex::new(State::default()));

    let state_task = state.clone();
    let handle = tokio::task::spawn_blocking(move || loop {
        let state = state_task.lock().unwrap();

        // publish buffer position to tf
        let mut transforms = vec![];
        if let Some(t) = state.buffer_transform.as_ref() {
            transforms.push(t.clone());
        }
        let tf_msg = TFMessage {
            transforms,
        };
        tf_pub.publish(&tf_msg).expect("could not publish");

        node.spin_once(std::time::Duration::from_millis(100));
    });

    tf_sub.for_each(|msg| {
        msg.transforms.iter().for_each(|t| {
            if t.child_frame_id == "locked_aruco_0" {
                println!("updated aruco");
                state.lock().unwrap().marker_0.replace(t.clone());
            }
            if t.child_frame_id == "locked_aruco_1" {
                println!("updated aruco 1");
                state.lock().unwrap().marker_1.replace(t.clone());
            }
            if t.child_frame_id == "table" {
                println!("updated table height");
                state.lock().unwrap().table_height
                    .replace(t.transform.translation.z);
            }

            {
                let mut state = state.lock().unwrap();
                if state.marker_0.is_some() && state.marker_1.is_some() &&
                    state.table_height.is_some() {
                    let marker_0 = &state.marker_0.as_ref().unwrap().transform;
                    let marker_1 = &state.marker_1.as_ref().unwrap().transform;
                    let table_height = state.table_height.as_ref().unwrap();

                    let diff_x = marker_1.translation.x - marker_0.translation.x;
                    let diff_y = marker_1.translation.y - marker_0.translation.y;
                    let yaw = diff_y.atan2(diff_x);

                    let mut transform = state.marker_0.as_ref().unwrap().clone();
                    transform.child_frame_id = "paper".into();

                    let rot = Quaternion::from(Euler {
                        x: Rad(0.0),
                        y: Rad(0.0),
                        z: Rad(yaw),
                    });

                    let rot2 = Quaternion::from(Euler {
                        x: Deg(180.0),
                        y: Deg(0.0),
                        z: Deg(0.0),
                    });

                    let gantry_q = rot * rot2;

                    transform.transform.rotation.w = gantry_q.s;
                    transform.transform.rotation.x = gantry_q.v.x;
                    transform.transform.rotation.y = gantry_q.v.y;
                    transform.transform.rotation.z = gantry_q.v.z;

                    // table height
                    transform.transform.translation.z = *table_height;

                    state.buffer_transform = Some(transform);
                }
            }

        });
        future::ready(())
    }).await;

    handle.await?;

    Ok(())
}


        // if msg.child_frame_id == "aruco_0" {
        //     update_or_set(msg.clone(), &mut state.lock().unwrap().marker_0);
        // }
        // if msg.child_frame_id == "aruco_1" {
        //     update_or_set(msg.clone(), &mut state.lock().unwrap().marker_1);
        // }

        // {
        //     let mut state = state.lock().unwrap();
        //     if state.marker_0.is_some() && state.marker_1.is_some() {
        //         let marker0 = state.marker_0.as_ref().unwrap().transform.clone();
        //         let marker1 = state.marker_1.as_ref().unwrap().transform.clone();

        //         let diff_x = marker1.translation.x - marker0.translation.x;
        //         let diff_y = marker1.translation.y - marker0.translation.y;
        //         let yaw = diff_y.atan2(diff_x);

        //         let mut new_transform = state.marker_1.as_ref().unwrap().clone();
        //         new_transform.child_frame_id = "facade_aruco".into();

        //         let rot = Quaternion::from(Euler {
        //             x: Rad(0.0),
        //             y: Rad(0.0),
        //             z: Rad(yaw),
        //         });

        //         let rot2 = Quaternion::from(Euler {
        //             x: Deg(180.0),
        //             y: Deg(0.0),
        //             z: Deg(0.0),
        //         });

        //         // set yaw and rotate around x to turn upside down.
        //         let new_q = rot * rot2;

        //         new_transform.transform.rotation.w = new_q.s;
        //         new_transform.transform.rotation.x = new_q.v.x;
        //         new_transform.transform.rotation.y = new_q.v.y;
        //         new_transform.transform.rotation.z = new_q.v.z;

        //         // set hardcoded height
        //         new_transform.transform.translation.z = 3.57;

        //         state.facade_transform = Some(new_transform);
        //     } else {
        //         state.facade_transform = None;
        //     }
        // }

        // if msg.child_frame_id == "aruco_2" {
        //     update_or_set(msg.clone(), &mut state.lock().unwrap().marker_2);
        // }

        // if msg.child_frame_id == "aruco_15" {
        //     update_or_set(msg.clone(), &mut state.lock().unwrap().marker_15);
        // }

        // {
        //     let mut state = state.lock().unwrap();
        //     if state.marker_15.is_some() && state.marker_2.is_some() {
        //         let marker15 = &state.marker_15.as_ref().unwrap().transform;
        //         let marker2 = &state.marker_2.as_ref().unwrap().transform;

        //         let diff_x = marker15.translation.x - marker2.translation.x;
        //         let diff_y = marker15.translation.y - marker2.translation.y;
        //         let yaw = diff_y.atan2(diff_x);

        //         // gantry position is marker15 position with this new rotation.
        //         let mut gantry_transform = state.marker_15.as_ref().unwrap().clone();
        //         gantry_transform.child_frame_id = "gantry_aruco".into();

        //         let rot = Quaternion::from(Euler {
        //             x: Rad(0.0),
        //             y: Rad(0.0),
        //             z: Rad(yaw),
        //         });

        //         let rot2 = Quaternion::from(Euler {
        //             x: Deg(180.0),
        //             y: Deg(0.0),
        //             z: Deg(0.0),
        //         });

        //         let gantry_q = rot * rot2;

        //         gantry_transform.transform.rotation.w = gantry_q.s;
        //         gantry_transform.transform.rotation.x = gantry_q.v.x;
        //         gantry_transform.transform.rotation.y = gantry_q.v.y;
        //         gantry_transform.transform.rotation.z = gantry_q.v.z;

        //         // hardcoded height
        //         gantry_transform.transform.translation.z = 1.93;

        //         state.gantry_transform = Some(gantry_transform);
        //     } else {
        //         state.gantry_transform = None;
        //     }
