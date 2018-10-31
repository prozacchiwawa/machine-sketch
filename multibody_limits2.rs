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
use std::collections::HashMap;
use std::rc::Rc;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, Ball, ShapeHandle};
use nphysics2d::joint::{FreeJoint, RevoluteJoint};
use nphysics2d::object::{BodyHandle, Material, Body, BodyMut};
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
    teeth: i16,
    motor: f32,
    pattern: Vec<bool>,
    subgears: Vec<Gear>
}

#[derive(Serialize, Deserialize)]
struct Pusher {
    name: String,
    source_x: f32,
    source_y: f32,
    push_away_x: f32,
    push_away_y: f32,
    source_plunger_width: f32,
    target_x: f32,
    target_y: f32,
    target_toward_x: f32,
    target_toward_y: f32,
    target_plunger_width: f32
}

#[derive(Serialize, Deserialize)]
struct Machine {
    name: String,
    gears: Vec<Gear>,
    pushers: Vec<Pusher>
}

const COLLIDER_MARGIN: f32 = 0.01;

enum ObjectType {
    GearType { body : BodyHandle }
}

struct ObjectData {
    name : String,
    objty : ObjectType
}

fn create_gear(
    parent : BodyHandle,
    gear : &Gear,
    collection : &mut HashMap<String,ObjectData>,
    world : &mut World<f32>) {
    let geom = ShapeHandle::new(Ball::new(gear.toothRadius));
    let centerGeom = ShapeHandle::new(Ball::new(gear.bodyRadius * 0.99));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    let mut gearSpin = RevoluteJoint::new(0.0);
    gearSpin.disable_min_angle();
    gearSpin.disable_max_angle();
    
    if (gear.motor != 0.0) {
        gearSpin.enable_angular_motor();
        gearSpin.set_desired_angular_motor_velocity(gear.motor);
        gearSpin.set_max_angular_motor_torque(5000.0);
    }

    let mut gearBody = world.add_multibody_link(
        parent,
        gearSpin,
        na::Vector2::new(gear.x, gear.y),
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

    let teeth = gear.teeth as usize;
    for i in 0usize..teeth {
        // Skip based on pattern
        if gear.pattern.len() > 0 && gear.pattern[i % gear.pattern.len()] {
            continue
        }
        
        let fi = i as f32;
        let fnum = gear.teeth as f32;
        // Setup the other links with revolute joints.
        let angle = (fi / fnum) * PI * 2.0;
        let dist = gear.toothRadius + gear.bodyRadius;
        let mut r = RevoluteJoint::new(angle);

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

    collection.insert(gear.name.clone(), ObjectData {
        name: gear.name.clone(),
        objty: ObjectType::GearType { body: gearBody }
    });
    
    for i in 0usize..(gear.subgears.len()) {
        create_gear(gearBody,&gear.subgears[i],collection,world);
    }
}

fn run(args : Vec<String>) -> std::result::Result<(), std::io::Error> {
    let mut file = File::open(args[1].clone())?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;

    let mut collection = HashMap::new();
    
    let machine : Machine = serde_json::de::from_str(&contents)?;

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

    world.set_contact_model(nphysics2d::solver::SignoriniModel::new());
    
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
    let mut parent = BodyHandle::ground();

    for i in 0usize..(machine.gears.len()) {
        create_gear(parent,&machine.gears[i],&mut collection, &mut world);
    }

    /*
     * Set up the testbed.
     */
    {
        let mut testbed = Testbed::new(world);
        testbed.add_callback(move |wld,gfx,time| {
            let world = wld.get_mut();
            for (name, odata) in &collection {
                println!("Object {}", name);
                match odata.objty {
                    ObjectType::GearType { body } => {
                        let theBody = world.body(body);
                        match theBody {
                            Body::Multibody(v) => {
                                for l in v.links() {
                                    println!("Pos {}", l.center_of_mass());
                                }
                            }
                            _ => { }
                        }
                    }
                }
            }
        });
        testbed.run();
    }

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
