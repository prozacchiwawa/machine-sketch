extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate serde;
extern crate serde_json;

#[macro_use]
extern crate serde_derive;

use std::env;
use std::fs::File;
use std::io::prelude::*;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, Ball, ShapeHandle};
use nphysics2d::joint::{FreeJoint, RevoluteJoint};
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use std::f32::consts::PI;
use std::f32;

use serde_json::Error;

#[derive(Serialize, Deserialize)]
struct Gear {
    name: String,
    bodyRadius: f32,
    toothRadius: f32,
    x: f32,
    y: f32,
    pattern: Vec<bool>
}

#[derive(Serialize, Deserialize)]
struct Machine {
    name: String,
    gears: Vec<Gear>
}

const COLLIDER_MARGIN: f32 = 0.01;

fn gear(
    parent : BodyHandle,
    num : usize,
    innerRad : f32,
    toothRad : f32,
    mountAt : na::Vector2<f32>,
    world : &mut World<f32>) {
    let geom = ShapeHandle::new(Ball::new(toothRad));
    let centerGeom = ShapeHandle::new(Ball::new(innerRad * 0.99));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    let mut gearSpin = RevoluteJoint::new(0.0);
    gearSpin.disable_min_angle();
    gearSpin.disable_max_angle();
    
    let mut gearBody = world.add_multibody_link(
        parent,
        gearSpin,
        mountAt,
        na::zero(),
        inertia,
        center_of_mass,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        centerGeom.clone(),
        gearBody,
        Isometry2::identity(),
        Material::default(),
    );

    for i in 0usize..num {
        let fi = i as f32;
        let fnum = num as f32;
        // Setup the other links with revolute joints.
        let angle = (fi / fnum) * PI * 2.0;
        let dist = toothRad + innerRad;
        let mut r = RevoluteJoint::new(angle);
        r.enable_min_angle(angle);
        r.enable_max_angle(angle);

        let posV = Vector2::new(
            dist * angle.sin(),
            dist * angle.cos() * -1.0,
        );
        let p = world.add_multibody_link(
            gearBody,
            r,
            posV,
            na::zero(),
            inertia,
            center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            p,
            Isometry2::identity(),
            Material::default(),
        );
    }
}

fn run(args : Vec<String>) -> std::result::Result<(), std::io::Error> {
    let mut file = File::open(args[1].clone())?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(na::zero());

    /*
     * Setup the ground.
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * -7.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Setup the multibody.
     */
    let rad = 0.2;
    let innerRad = 3.0;
    let num = 20;
    let mut parent = BodyHandle::ground();
    let gearAt = na::zero();

    gear(parent, num, innerRad, rad, gearAt, &mut world);

    /*
     * Set up the testbed.
     */
    let testbed = Testbed::new(world);
    testbed.run();

    return Ok(())
}

fn main() {
    // Read the file
    let args : Vec<String> = std::env::args().collect();
    if (args.len() <= 1) {
        println!("Usage: multibody_limits2 [gears.json]");
        return
    }

    match run(args) {
        Ok(_)  => println!("done"),
        Err(e) => println!("error {:?}", e),
    }
}
