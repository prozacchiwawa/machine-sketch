extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate serde;
extern crate serde_json;
extern crate downcast;

#[macro_use]
extern crate serde_derive;

use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::collections::HashMap;
use std::rc::Rc;

use na::{Isometry2, Vector2, Unit, Point2};
use ncollide2d::shape::{Cuboid, Ball, ShapeHandle};
use nphysics2d::joint::{FreeJoint, RevoluteJoint, PrismaticJoint, MouseConstraint, ConstraintHandle, FixedJoint};
use nphysics2d::object::{BodyHandle, Material, Body, BodyMut};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::math::Isometry;
use nphysics_testbed2d::Testbed;
use std::f32::consts::PI;
use std::f32;
use downcast::TypeMismatch;
use serde_json::Error;

#[derive(Serialize, Deserialize, Clone)]
struct Gear {
    name: String,
    bodyRadius: f32,
    toothRadius: f32,
    x: f32,
    y: f32,
    teeth: i16,
    motor: f32,
    pattern: Vec<bool>,
    subs: Vec<MachinePart>
}

#[derive(Serialize, Deserialize, Clone)]
enum ConstraintType {
    Free,
    Fixed,
    Spring(f32),
}

#[derive(Serialize, Deserialize, Clone)]
struct Brick {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    hold: ConstraintType,
    parts: Vec<MachinePart>
}

#[derive(Serialize, Deserialize, Clone)]
struct PushEnd {
    source_x: f32,
    source_y: f32,
    push_away_x: f32,
    push_away_y: f32,
    plunger_size: f32,
    parts: Vec<MachinePart>
}

#[derive(Serialize, Deserialize, Clone)]
struct Pusher {
    name: String,
    left: PushEnd,
    right: PushEnd
}

#[derive(Serialize, Deserialize, Clone)]
struct GearCouple {
    from: String,
    toward: String,
    multiple: f32
}

#[derive(Serialize, Deserialize, Clone)]
enum MachinePart {
    GearPart(Gear),
    BrickPart(Brick),
    PusherPart(Pusher),
    GearCouple(GearCouple)
}

#[derive(Serialize, Deserialize)]
struct Machine {
    name: String,
    parts: Vec<MachinePart>
}

const COLLIDER_MARGIN: f32 = 0.0001;

enum ObjectType {
    GearType { body : BodyHandle },
    PusherType
    { pusher : Pusher,
      left : BodyHandle,
      right : BodyHandle,
      mcleft : ConstraintHandle,
      mcright : ConstraintHandle
    }
}

struct ModifyPusher {
    pusher : Pusher,
    mcright : ConstraintHandle,
    center : Point2<f32>
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
        gearSpin.set_max_angular_motor_torque(100.0);
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
    
    for i in 0usize..(gear.subs.len()) {
        create_part(gearBody, &gear.subs[i], collection, world);
    }
}

fn vdist(v : Vector2<f32>) -> f32 {
    return (v.x * v.x + v.y * v.y).sqrt();
}

fn create_pusher(
    parent : BodyHandle,
    pusher : &Pusher,
    collection : &mut HashMap<String,ObjectData>,
    world : &mut World<f32>) {
    /* Left */
    let geomLeft = ShapeHandle::new(Cuboid::new(Vector2::repeat(pusher.left.plunger_size)));
    let inertia = geomLeft.inertia(1.0);
    let center_of_mass = geomLeft.center_of_mass();
    let towardLeft = Vector2::new(
        pusher.left.push_away_x - pusher.left.source_x,
        pusher.left.push_away_y - pusher.left.source_y
    );

    let pusherLeftDist = vdist(towardLeft);
    let mut pushSlideLeftJoint =
        PrismaticJoint::new(Unit::new_normalize(towardLeft), pusherLeftDist);
    let pusherLeftHandle = world.add_multibody_link(
        parent,
        pushSlideLeftJoint,
        Vector2::new(pusher.left.source_x, pusher.left.source_y),
        na::zero(),
        inertia,
        center_of_mass
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geomLeft.clone(),
        pusherLeftHandle,
        Isometry2::identity(),
        Material::default(),
    );

    let mcleft = MouseConstraint::new(
        parent,
        pusherLeftHandle,
        na::Point2::new(pusher.left.source_x, pusher.left.source_y), 
        center_of_mass,
        0.5
    );

    let mcleftHandle = world.add_constraint(mcleft);

    /* Right */
    let geomRight = ShapeHandle::new(Cuboid::new(Vector2::repeat(pusher.right.plunger_size)));
    let inertia = geomRight.inertia(1.0);
    let center_of_mass = geomRight.center_of_mass();
    let towardRight = Vector2::new(
        pusher.right.push_away_x - pusher.right.source_x,
        pusher.right.push_away_y - pusher.right.source_y
    );

    let pusherRightDist = vdist(towardRight);
    let mut pushSlideRightJoint =
        PrismaticJoint::new(Unit::new_normalize(towardRight), pusherRightDist);
    let pusherRightHandle = world.add_multibody_link(
        parent,
        pushSlideRightJoint,
        Vector2::new(pusher.right.source_x, pusher.right.source_y),
        na::zero(),
        inertia,
        center_of_mass
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geomRight.clone(),
        pusherRightHandle,
        Isometry2::identity(),
        Material::default(),
    );

    let mcright = MouseConstraint::new(
        parent,
        pusherRightHandle,
        na::Point2::new(pusher.right.source_x, pusher.right.source_y), 
        center_of_mass,
        0.5
    );

    let mcrightHandle = world.add_constraint(mcright);
    
    collection.insert(pusher.name.clone(), ObjectData {
        name: pusher.name.clone(),
        objty: ObjectType::PusherType
        { pusher: pusher.clone(),
          left: pusherLeftHandle,
          right: pusherRightHandle,
          mcleft: mcleftHandle,
          mcright: mcrightHandle
        }
    });

    for i in 0usize..(pusher.left.parts.len()) {
        create_part(
            pusherLeftHandle,
            &pusher.left.parts[i],
            collection,
            world
        );
    }
    
    for i in 0usize..(pusher.right.parts.len()) {
        create_part(
            pusherRightHandle,
            &pusher.right.parts[i],
            collection,
            world
        );
    }    
}

fn create_brick(
    parent : BodyHandle,
    brick : &Brick,
    collection : &mut HashMap<String,ObjectData>,
    world : &mut World<f32>) {
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(brick.width, brick.height)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();
    
    let mut fixedJoint = FixedJoint::new(Isometry::new(na::zero(), na::zero()));
    let brickHandle = world.add_multibody_link(
        parent,
        fixedJoint,
        Vector2::new(brick.x, brick.y),
        na::zero(),
        inertia,
        center_of_mass
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        brickHandle,
        Isometry2::identity(),
        Material::default(),
    );

    for i in 0usize..(brick.parts.len()) {
        create_part(
            brickHandle,
            &brick.parts[i],
            collection,
            world
        );
    }
}

fn create_part(
    parent : BodyHandle,
    part : &MachinePart,
    collection : &mut HashMap<String,ObjectData>,
    world : &mut World<f32>) {
    match part {
        MachinePart::GearPart(gear) => {
            create_gear(parent, &gear, collection, world);
        }
        MachinePart::PusherPart(pusher) => {
            create_pusher(parent, &pusher, collection, world);
        }
        MachinePart::BrickPart(brick) => {
            create_brick(parent, &brick, collection, world);
        }
        _ => { }
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
    let parent = BodyHandle::ground();

    for i in 0usize..(machine.parts.len()) {
        create_part(parent, &machine.parts[i], &mut collection, &mut world);
    }

    /*
     * Set up the testbed.
     */
    {
        let mut testbed = Testbed::new(world);
        testbed.add_callback(move |wld,gfx,time| {
            let mut world = wld.get_mut();
            for (name, odata) in &collection {
                // println!("Object {}", name);
                match odata.objty {
                    ObjectType::GearType { body } => {
                        // Nothing
                    }
                    ObjectType::PusherType { ref pusher, left, right, mcleft, mcright } => {
                        let mut ch = None;
                        {
                            let theBody = world.body(left);
                            match theBody {
                                Body::Multibody(v) => {
                                    for l in v.links() {
                                        // println!("Pos {}", l.center_of_mass());
                                        ch = Some(ModifyPusher {
                                            pusher: pusher.clone(),
                                            mcright: mcright,
                                            center: l.center_of_mass()
                                        });
                                        break
                                    }
                                }
                                _ => { }
                            }
                        };
                        match ch {
                            Some(pmove) => {
                                // Determine percentage distance of left
                                let left_source =
                                    Vector2::new(pusher.left.source_x, pusher.left.source_y);
                                let left_target =
                                    Vector2::new(pusher.left.push_away_x, pusher.left.push_away_y);
                                let left_center = pmove.center.coords;
                                let left_mag = vdist(Vector2::new(left_center.x - pusher.left.source_x, left_center.y - pusher.left.source_y));
                                let left_full = vdist(Vector2::new(left_target.x - left_source.x, left_target.y - left_source.y));
                                
                                // XXX todo left_center - source_xy
                                let travel_distance = left_mag / left_full;
                                // println!("{} travel distance {}", pusher.name.clone(), travel_distance);
                                let right_source =
                                    Vector2::new(pusher.right.source_x, pusher.right.source_y);
                                let right_target =
                                    Vector2::new(pusher.right.push_away_x, pusher.right.push_away_y);
                                let right_travel_to =
                                    Vector2::new(
                                        pusher.right.source_x + (pusher.right.push_away_x - pusher.right.source_x) * travel_distance,
                                        pusher.right.source_y + (pusher.right.push_away_y - pusher.right.source_y) * travel_distance
                                    );
                                
                                let right_constraint = world.constraint_mut(mcright);
                                let right_mouse : Result<& mut MouseConstraint<f32>, downcast::TypeMismatch> = right_constraint.downcast_mut();
                                match right_mouse {
                                    Ok(rm) => {
                                        rm.set_anchor_2(na::Point2::new(right_travel_to.x, right_travel_to.y));
                                    }
                                    _ => {}
                                }
                            }
                            _ => {}
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
