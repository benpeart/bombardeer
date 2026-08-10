include <BOSL2/std.scad>
include <BOSL2/gears.scad>

$fn = 64; // High rendering resolution for round holes

// --- System Parameters ---
mod         = 2;     // Tooth size (Module)
starts      = 1;     // 1 thread = 30:1 reduction, self-locking
worm_d      = 50;    // Pitch diameter of the worm screw (48mm)
worm_l      = 50;    // Length of the worm screw (40mm)
gear_radius = 50;    // Target pitch radius (330mm)
teeth       = (gear_radius * 2) / mod; // 350 teeth for 700mm pitch diameter

// --- Shaft Fit Dimensions ---
nema_d      = 5.2;   // NEMA 17 5mm shaft diameter (+0.2mm clearance for 3D printing)
nema_flat   = 4.6;   // D-shaft flat dimension (+0.1mm clearance)
tslot_size  = 20.0;  // 2020 extrusion width (+0.0mm clearance)

// =================================================================
// 1. THE WORM SCREW (WITH NEMA 17 D-SHAFT HOLE)
// =================================================================
difference() {
    // Generate base worm without internal shaft hole
    worm(
        mod = mod,
        d = worm_d,
        l = worm_l,
        starts = starts,
        shaft_diam = 0
    );
    
    // Subtract NEMA 17 D-Shaft profile through center
    translate([0, 0, -1])
        linear_extrude(h = worm_l)
            intersection() {
                // 5mm Round Bore
                circle(d = nema_d);
                
                // Flat cut side for D-profile
                translate([-nema_d/2, -nema_d/2])
                    square([nema_d, nema_flat]);
            }
}

// =================================================================
// 2. CALCULATE EXACT CENTER DISTANCE
// =================================================================
dist = worm_dist(mod=mod, d=worm_d, starts=starts, teeth=teeth);

// =================================================================
// 3. THE WORM GEAR (WITH 2020 T-SLOT SQUARE HOLE)
// =================================================================
translate([dist+10, 0, 0])
    rotate([90, 9, 0]) // Orient 90 degrees to mesh with worm thread
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
            translate([0, 0, -20])
                linear_extrude(h = worm_l)
                    square([tslot_size, tslot_size], center = true);
        }