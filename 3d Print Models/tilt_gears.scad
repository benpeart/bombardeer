include <BOSL2/std.scad>
include <BOSL2/gears.scad>

$fn = 64; // High rendering resolution for circular bores

// --- VIEW / PRINT MODE ---
// Options: "assembly" (view both in position), "screw" (print worm screw), "gear" (print worm gear)
mode = "assembly"; 

// --- System Parameters ---
mod         = 2;     // Tooth size (Module)
starts      = 1;     // 1 start thread (50:1 reduction, self-locking)
worm_d      = 50;    // Pitch diameter of worm screw (mm)
worm_l      = 50;    // Length of worm screw (mm)
gear_radius = 50;    // Pitch radius of worm gear (mm)
teeth       = (gear_radius * 2) / mod; // 50 teeth

// --- Shaft & Fastener Dimensions ---
nema_d        = 5.2;   // NEMA 17 5mm shaft diameter (+0.2mm clearance)
nema_flat     = 4.6;   // D-shaft flat dimension (+0.1mm clearance)
shaft_len     = 15.0;  // Length of the NEMA 17 motor shaft (mm)
tslot_size    = 20.3;  // 2020 extrusion width (+0.3mm clearance for 3D printing)

// --- M3 Heat-Set Insert Dimensions ---
m3_insert_dia    = 3.9;  // Heat-set insert outer diameter (mm)
m3_insert_len    = 8.0;  // Length of the brass insert (mm)
m3_screw_thru    = 3.2;  // Screw tip clearance hole between insert pocket and D-shaft for M3 x 20mm screw
insert_overdepth = 1.0;  // Extra clearance depth past the insert so screws don't bottom out

// =================================================================
// POSITION CALCULATIONS
// =================================================================
// 1. Exact gear mesh center distance
center_distance = worm_dist(mod=mod, d=worm_d, starts=starts, teeth=teeth);

// 2. Calculate Set-Screw Z-position:
// Axial pitch of worm thread
axial_pitch   = PI * mod * starts;           // ~6.28mm
// Half-pitch offset moves from crest to root
root_offset   = axial_pitch / 2;             // ~3.14mm
// Ideal center point of the 15mm motor shaft inserted from the bottom
target_z      = -worm_l / 2 + shaft_len / 2; // -25mm + 7.5mm = -17.5mm
// Snap target_z to the nearest thread valley (root)
screw_z       = round((target_z - root_offset) / axial_pitch) * axial_pitch + root_offset; // ~ -15.71mm

// 3. Worm root radius & Insert seat depth
worm_root_r   = (worm_d / 2) - (1.25 * mod);                 // Root radius (~22.5mm)
pocket_bottom = worm_root_r - (m3_insert_len + insert_overdepth); // Base of heat-set pocket

/* ========================================================================
   MAIN DISPLAY SWITCH
   ======================================================================== */

if (mode == "assembly") {
    // Render Worm Gear at origin
    render_gear();
    
    // Render Worm Screw at exact pitch center distance
    translate([center_distance + 10, 0, 0])
        worm_screw();

} else if (mode == "screw") {
    worm_screw();

} else if (mode == "gear") {
    render_gear();
}


/* ========================================================================
   MODULE DEFINITIONS
   ======================================================================== */

// 1. THE WORM SCREW (WITH NEMA 17 D-SHAFT & HEAT-SET INSERT POCKET)
module worm_screw() {
    difference() {
        // Base worm screw centered at Z = 0 (spans Z = -25 to +25)
        worm(
            mod = mod,
            d = worm_d,
            l = worm_l,
            starts = starts,
            shaft_diam = 0
        );
        
        // Subtract NEMA 17 D-Shaft profile through full center length
        linear_extrude(h = worm_l + 2, center = true)
            intersection() {
                // 5mm Round Bore
                circle(d = nema_d);
                
                // Flat cut side for D-profile (flat sits at +Y = nema_flat - nema_d/2)
                translate([-nema_d / 2, -nema_d / 2])
                    square([nema_d, nema_flat]);
            }

        // M3 Heat-Set Insert Pocket & Screw Channel
        // Positioned at screw_z (~ -15.71mm) directly over the shaft flat
        translate([0, 0, screw_z])
            rotate([-90, 0, 0]) {
                // 1. Screw tip pass-through hole into the D-shaft flat
                cyl(d = m3_screw_thru, h = worm_root_r, anchor = BOTTOM);

                // 2. 3.9mm diameter heat-set insert bore (starts at worm root and seats down)
                translate([0, 0, pocket_bottom])
                    cyl(d = m3_insert_dia, h = (worm_d / 2) - pocket_bottom + 5, anchor = BOTTOM);
            }
    }
}

// 2. THE WORM GEAR (WITH 2020 T-SLOT SHAFT BORE)
module render_gear() {
    rotate([90, -2, 0]) // 90 deg mesh axis orientation -2 deg thread lead phase to line them up
        difference() {
            // Generate base gear wheel without internal shaft hole
            worm_gear(
                mod = mod,
                teeth = teeth,
                worm_diam = worm_d,
                worm_starts = starts,
                shaft_diam = 0
            );
            
            // Subtract 2020 T-Slot Square Profile through center
            linear_extrude(h = worm_l + 20, center = true)
                square([tslot_size, tslot_size], center = true);
        }
}