extern crate stdweb;
#[macro_use]
extern crate downcast_rs;

use std::cell::RefCell;
use std::rc::Rc;
use stdweb::traits::*;
use stdweb::web::event::{ClickEvent, ContextMenuEvent, MouseButton, ResizeEvent};
use stdweb::web::{window, IEventTarget};

mod body;
mod manifold;
mod math;
mod rand;
mod scene;
use scene::*;

fn game_loop(scene: Rc<RefCell<Scene>>) {
    let window = stdweb::web::window();
    window.request_animation_frame(move |_| {
        scene.borrow_mut().step();
        // Here, the first borrow_mut has been dropped, so it's ok to borrow it again
        scene.borrow_mut().render();
        game_loop(scene.clone());
    });
}

fn main() {
    stdweb::initialize();
    // Here we need `scene` to be mutable shared
    let scene = Rc::new(RefCell::new(Scene::new()));
    scene.borrow_mut().resize(
        (window().inner_width() as f64 * 0.8) as u32,
        (window().inner_width() as f64 * 0.8) as u32,
    );
    // `add_event_listener` accepts F: FnMut(T) + 'static, so we actually cannot borrow `scene` since F can live as long as static. We have to move it.
    scene.borrow().canvas.canvas.add_event_listener({
        let scene = scene.clone();
        move |event: ContextMenuEvent| {
            event.prevent_default();
            scene
                .borrow_mut()
                .add_circle(event.offset_x(), event.offset_y());
            //console!(
            //log,
            //"right click at %f, %f",
            //event.offset_x(),
            //event.offset_y()
            //);
        }
    });
    scene.borrow().canvas.canvas.add_event_listener({
        let scene = scene.clone();
        move |event: ClickEvent| match event.button() {
            MouseButton::Left => {
                scene
                    .borrow_mut()
                    .add_polygon(event.offset_x(), event.offset_y());
                //console!(
                //log,
                //"left click at %f, %f",
                //event.offset_x(),
                //event.offset_y()
                //);
            }
            _ => {}
        }
    });
    stdweb::web::window().add_event_listener({
        let scene = scene.clone();
        move |_: ResizeEvent| {
            // console!(log, "resize event!");
            scene.borrow_mut().resize(
                (window().inner_width() as f64 * 0.8) as u32,
                (window().inner_width() as f64 * 0.8) as u32,
            );
        }
    });

    game_loop(scene);

    stdweb::event_loop();
}
