/* ========================================================================
   LAZY SUSAN RING GEAR & NEMA 17 PINION SET (BOSL2)
   ========================================================================
   - Ring Gear: 115 Teeth (281.25mm Root Dia, 287.5mm Pitch Dia)
   - Pinion Gear: 15 Teeth (37.5mm Pitch Dia)
   - Pinion Root Valley Alignment: 0° (BOSL2 centers gap at 0°)
   - M3 Set Screw: True radial bore, recessed 3.5mm below tooth root valley
   - Calculated Center Distance: 162.5mm
   ======================================================================== */

include <BOSL2/std.scad>
include <BOSL2/gears.scad>

$fn = 96; // Smooth curve resolution

/* --- VIEW / PRINT MODE --- */
// Options: "assembly" (view both in position), "ring" (print ring), "pinion" (print pinion)
mode = "assembly"; 

/* --- COMMON GEAR PARAMETERS --- */
module_val      = 2.5;   // Gear Module (2.5mm pitch)
pressure_angle  = 20;    // Standard 20-degree pressure angle
gear_height     = 18;    // Uniform gear thickness in Z (mm)

/* --- RING GEAR SPECIFICATIONS --- */
ring_teeth      = 115;   // 115 Teeth -> Root Dia = 281.25mm, Pitch Dia = 287.5mm
inner_bore_dia  = 180;   // Hollow center bore through full height (mm)
mount_radius    = 110.0; // Hole circle radius for Lazy Susan / T-slot (mm)

/* --- PINION GEAR SPECIFICATIONS (NEMA 17) --- */
pinion_teeth    = 15;    // 15 Teeth -> Pitch Dia = 37.5mm
nema_shaft_dia  = 5.2;   // 5mm shaft clearance (5.2mm for 3D printing tolerances)
nema_flat_dist  = 4.6;   // D-flat distance from rounded back (mm)

// Center of the root valley is 1/4 of the full tooth pitch
valley_angle = 360.0 / (4 * pinion_teeth); // (360 / 60) = 6.0° for 15 teeth

/* --- M3 COUNTERBORE FASTENER DIMENSIONS --- */
m3_screw_dia    = 3.2;   // M3 set-screw shank clearance hole (mm)
m3_cbore_dia    = 6.5;   // Counterbore pocket diameter for screw head & tool (mm)
m3_head_height  = 3.5;   // Recess depth below tooth root valley (mm)

/* --- CALCULATED CENTER DISTANCE --- */
// Pitch Radius (Ring) + Pitch Radius (Pinion)
center_distance = (module_val * (ring_teeth + pinion_teeth)) / 2.0; // 162.5mm


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

        // Full-Height Donut Center Cutout
        translate([0, 0, -1])
            cyl(d = inner_bore_dia, h = gear_height + 2, anchor = BOTTOM);

        // Bottom Face: 4x M5 Heat-Set Inserts (45°, 135°, 225°, 315°)
        for (angle = [45, 135, 225, 315]) {
            zrot(angle)
                translate([mount_radius, 0, -0.1])
                    cyl(d = 6.0, h = 10.0, anchor = BOTTOM);
        }

        // Top Face: 2x M5 Counterbored 2020 T-Slot Holes (0°, 180°)
        for (angle = [0, 180]) {
            zrot(angle) {
                translate([mount_radius, 0, 0]) {
                    // M5 Shank Clearance Hole
                    translate([0, 0, -1])
                        cyl(d = 5.5, h = gear_height + 2, anchor = BOTTOM);
                    
                    // Recessed Counterbore for Screw Head (from bottom)
                    translate([0, 0, -0.1])
                        cyl(d = 10.0, h = 10.1, anchor = BOTTOM);
                }
            }
        }
    }
}

// 2. NEMA 17 STEPPER PINION GEAR (VALLEY-ALIGNED & TRUE RADIAL COUNTERBORE)
module nema17_pinion() {
    pinion_pitch_r = (module_val * pinion_teeth) / 2.0;               // 18.75mm
    pinion_root_r  = pinion_pitch_r - (1.25 * module_val);            // 15.625mm
    pinion_shelf_r = pinion_root_r - m3_head_height;                  // 12.125mm recessed shelf
    pinion_outer_r = (module_val * (pinion_teeth + 2)) / 2.0;         // 21.25mm
    
    // Position of D-shaft flat relative to origin
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

        // D-Shaft Bore (Aligned with valley_angle)
        zrot(valley_angle)
            translate([0, 0, -1])
                d_shaft_bore(h = gear_height + 2);

        // M3 Recessed Counterbored Set-Screw Bore
        // Uses pure radial ray from [0,0,mid_z] outward along valley_angle
        zrot(valley_angle) {
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