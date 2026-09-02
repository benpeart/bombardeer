/* ========================================================================
   LAZY SUSAN RING GEAR (155mm TURNTABLE) & NEMA 17 PINION SET (BOSL2)
   ========================================================================
   - Lazy Susan: 155mm Square Plate (219.2mm diagonal)
   - Hole Spacing: 136mm x 136mm Square Pitch (Radius: 96.17mm)
   - Center Bore: 120.0mm Diameter
   - Gear Height: 10.0mm
   - Fully parametric counterbore & insert depths relative to gear_height
   ======================================================================== */

include <BOSL2/std.scad>
include <BOSL2/gears.scad>

$fn = 64; // Smooth curve resolution

/* --- VIEW / PRINT MODE --- */
// Options: "assembly" (view both in position), "ring" (print ring), "pinion" (print pinion)
mode = "assembly"; 

/* --- COMMON GEAR PARAMETERS --- */
module_val      = 2.5;   // Gear Module (2.5mm pitch)
pressure_angle  = 20;    // Standard 20-degree pressure angle
gear_height     = 15.0;  // Uniform gear thickness in Z (mm)

/* --- RING GEAR SPECIFICATIONS (155mm TURNTABLE) --- */
//ring_teeth      = 90;    // 90 Teeth -> Pitch Dia = 225.0mm, Root Dia = 218.75mm, Outer Dia = 230.0mm
ring_teeth      = 100;   // 100 Teeth -> Pitch Dia = 250.0mm, Root Dia = 218.75mm, Outer Dia = 255.0mm
inner_bore_dia  = 120.0; // Turntable center hole cutout (mm)

// 136mm square hole spacing: Radius = sqrt(68^2 + 68^2) = 96.167mm
ls_hole_pitch   = 136.0; 
mount_radius    = ls_hole_pitch / sqrt(2); // ~96.17mm

/* --- PINION GEAR SPECIFICATIONS (NEMA 17) --- */
pinion_teeth    = 15;    // 15 Teeth -> Pitch Dia = 37.5mm, Outer Dia = 42.5mm
//pinion_teeth    = 25;    // 25 Teeth -> Pitch Dia = 62.5mm, Outer Dia = 67.5mm
nema_shaft_dia  = 5.2;   // 5mm shaft clearance (+0.2mm for 3D printing)
nema_flat_dist  = 4.6;   // D-flat distance from rounded back (mm)

// Valley Center Alignment: 1/4 of the tooth pitch (360 / (4 * 15) = 6.0°)
pinion_valley_ang = 360.0 / (4 * pinion_teeth); // 6.0 degrees

/* --- FASTENER & COUNTERBORE SPECIFICATIONS --- */
// 1. Bottom Face M5 Heat-Set Inserts (Lazy Susan)
insert_hole_dia = 6.18;   // M5 heat-set insert pocket diameter (mm)
insert_depth    = min(6.0, gear_height - 2.0); // 6.0mm depth (leaves 4mm solid roof at 10mm height)

// 2. Top Face M5 2020 T-Slot Fasteners
m5_clearance_d  = 5.5;   // M5 2020 T-slot shank clearance hole (mm)
m5_head_cbore_d = 10.0;  // M5 head counterbore diameter (mm)
m5_shelf_thick  = 3.5;   // Solid plastic clamping shelf thickness left at top (mm)
m5_head_recess  = gear_height - m5_shelf_thick; // Calculated counterbore depth from bottom (~6.5mm)

// 3. Pinion M3 Set-Screw Fasteners
m3_screw_dia    = 3.2;   // M3 set-screw shank clearance hole (mm)
m3_cbore_dia    = 6.5;   // Flat-bottom counterbore diameter for M3 head (mm)
m3_head_height  = 3.5;   // Recess depth below tooth root valley (mm)

/* --- CALCULATED CENTER DISTANCE --- */
center_distance = (module_val * (ring_teeth + pinion_teeth)) / 2.0; // 131.25mm


/* ========================================================================
   MAIN DISPLAY SWITCH
   ======================================================================== */

if (mode == "assembly") {
    // Render Ring Gear at origin
    ring_gear();
    
    // Render NEMA 17 Pinion mesh aligned at exact pitch center distance
    translate([center_distance + 10, 0, 0])
        nema17_pinion();

} else if (mode == "ring") {
    ring_gear();

} else if (mode == "pinion") {
    nema17_pinion();
}


/* ========================================================================
   MODULE DEFINITIONS
   ======================================================================== */

// 1. LAZY SUSAN RING GEAR
module ring_gear() {
    difference() {
        // Base Gear Body
        spur_gear(
            mod = module_val,
            teeth = ring_teeth,
            thickness = gear_height,
            pressure_angle = pressure_angle,
            anchor = BOTTOM
        );

        // Full-Height 120mm Center Cutout
        translate([0, 0, -1])
            cyl(d = inner_bore_dia, h = gear_height + 2, anchor = BOTTOM);

        // Bottom Face: 4x M5 Heat-Set Inserts for 136mm x 136mm Lazy Susan
        for (angle = [45, 135, 225, 315]) {
            zrot(angle)
                translate([mount_radius, 0, -0.1])
                    cyl(d = insert_hole_dia, h = insert_depth + 0.1, anchor = BOTTOM);
        }

        // Top/Through: 45° Offset M5 Counterbored Holes for 2020 T-Slot
        for (angle = [0, 180]) {
            zrot(angle) {
                translate([mount_radius + 10, 0, 0]) {
                    // Full M5 Shank Clearance Through-Hole
                    translate([0, 0, -1])
                        cyl(d = m5_clearance_d, h = gear_height + 2, anchor = BOTTOM);
                    
                    // Parametric Flat Counterbore (leaves m5_shelf_thick solid material)
                    translate([0, 0, -0.1])
                        cyl(d = m5_head_cbore_d, h = m5_head_recess + 0.1, anchor = BOTTOM);
                }
            }
            zrot(angle) {
                translate([mount_radius - 15, 0, 0]) {
                    // Full M5 Shank Clearance Through-Hole
                    translate([0, 0, -1])
                        cyl(d = m5_clearance_d, h = gear_height + 2, anchor = BOTTOM);
                    
                    // Parametric Flat Counterbore (leaves m5_shelf_thick solid material)
                    translate([0, 0, -0.1])
                        cyl(d = m5_head_cbore_d, h = m5_head_recess + 0.1, anchor = BOTTOM);
                }
            }
        }
    }
}

// 2. NEMA 17 STEPPER PINION GEAR (VALLEY-ALIGNED & TRUE RADIAL RECESSED COUNTERBORE)
module nema17_pinion() {
    pinion_pitch_r = (module_val * pinion_teeth) / 2.0;               // 18.75mm
    pinion_root_r  = pinion_pitch_r - (1.25 * module_val);            // 15.625mm
    pinion_shelf_r = pinion_root_r - m3_head_height;                  // 12.125mm recessed shelf
    pinion_outer_r = (module_val * (pinion_teeth + 2)) / 2.0;         // 21.25mm
    flat_x_pos     = nema_flat_dist - (nema_shaft_dia / 2.0);         // 2.0mm

    difference() {
        // Main Pinion Body
        spur_gear(
            mod = module_val,
            teeth = pinion_teeth,
            thickness = gear_height,
            pressure_angle = pressure_angle,
            anchor = BOTTOM
        );

        // D-Shaft Bore (Aligned with valley angle)
        zrot(pinion_valley_ang)
            translate([0, 0, -1])
                d_shaft_bore(h = gear_height + 2);

        // M3 Recessed Counterbore Set-Screw Bore (Centered at mid-height Z = gear_height / 2)
        zrot(pinion_valley_ang) {
            translate([0, 0, gear_height / 2.0]) {
                // 1. M3 Shank Through-Hole (from D-flat out to recessed shelf)
                translate([flat_x_pos, 0, 0])
                    yrot(90)
                        cyl(d = m3_screw_dia, h = pinion_shelf_r - flat_x_pos, anchor = BOTTOM);

                // 2. Flat Counterbore Pocket (from recessed shelf out past tooth crests)
                translate([pinion_shelf_r, 0, 0])
                    yrot(90)
                        cyl(d = m3_cbore_dia, h = (pinion_outer_r - pinion_shelf_r) + 5, anchor = BOTTOM);
            }
        }
    }
}

// Helper: Standard NEMA 17 D-Shaft Cutout (Flat centered symmetrically on +X)
module d_shaft_bore(h) {
    flat_x = nema_flat_dist - (nema_shaft_dia / 2.0); // 2.0mm from center
    difference() {
        // Round shaft clearance
        cyl(d = nema_shaft_dia, h = h, anchor = BOTTOM);
        
        // Symmetrical D-flat cut on +X side
        translate([flat_x + (nema_shaft_dia / 2.0), 0, h / 2.0])
            cube([nema_shaft_dia, nema_shaft_dia * 2, h + 0.2], center = true);
    }
}