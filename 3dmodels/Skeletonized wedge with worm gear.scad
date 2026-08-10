include <BOSL2/std.scad>
include <BOSL2/gears.scad>

$fn = 64; // High rendering resolution

// --- Gear Parameters ---
mod          = 2;     // Tooth size (Module)
starts       = 1;     // 1 thread = 30:1 reduction, self-locking
gear_radius  = 50;   // Target pitch radius (330mm)
teeth        = (gear_radius * 2) / mod; // 350 teeth for 700mm pitch diameter

// --- Pitch Range / Sector Wedge Parameters ---
pitch_range  = 40;    // Required travel range (degrees) - TRY CHANGING THIS!
margin_angle = 5;     // Extra safety margin (degrees)
sector_angle = pitch_range + margin_angle; // Total sector angle

// --- Worm Screw Parameters (NEMA 17 Flat Side Clearance) ---
worm_d       = 48;    // Pitch diameter clearing 21.15mm NEMA 17 flat side (mm)
worm_l       = 45;    // Length of the worm screw (mm)

// --- Central Hub & Shaft Fit Dimensions ---
tslot_size   = 20.2;  // 2020 extrusion width (+0.2mm clearance)
hub_diam     = 42.0;  // Full 360-degree hub diameter around 2020 rail (mm)
hub_thickness= 10.0;  // Thickness of mounting hub along 2020 rail (25mm)
nema_d       = 5.2;   // NEMA 17 5mm D-shaft diameter (+0.2mm clearance)
nema_flat    = 4.6;   // D-shaft flat dimension (+0.1mm clearance)

// =================================================================
// PARAMETRIC SKELETONIZING / LIGHTENING PARAMETERS
// =================================================================
edge_wall_angle   = 3.5;  // Solid wall margin along left/right sector edges (degrees)
spine_half_angle  = 2.0;  // Half-angle of central stiffening spine (degrees, total width = 4 deg)

num_radial_steps  = 3;    // Number of pocket sections along arm length
radial_rib_thick  = 10.0; // Thickness of transverse cross-ribs (mm)
hub_wall_thick    = 8.0;  // Solid plastic ring around central hub (mm)
rim_wall_thick    = 25.0; // Solid plastic arc under gear teeth (mm)

// --- Derived Radial Calculations ---
r_start      = (hub_diam / 2) + hub_wall_thick;
r_end        = gear_radius - rim_wall_thick;
total_span_r = r_end - r_start;
pocket_r_len = (total_span_r - ((num_radial_steps - 1) * radial_rib_thick)) / num_radial_steps;

// --- Derived Angular Calculations ---
max_pocket_half_angle = (sector_angle / 2) - edge_wall_angle;
has_dual_pockets      = (max_pocket_half_angle - spine_half_angle) > 2.0; // Needs >=2 deg pocket width

// =================================================================
// HELPER MODULE: Lightening Window Pocket
// =================================================================
module pocket_cutout(r1, r2, a1, a2, h) {
    rotate([0, 0, a1])
        rotate_extrude(angle = a2 - a1)
            translate([r1, -h/2 - 1])
                square([r2 - r1, h + 2]);
}

// =================================================================
// 1. THE WORM SCREW (NEMA 17 D-SHAFT BORE)
// =================================================================
difference() {
    worm(
        mod = mod,
        d = worm_d,
        l = worm_l,
        starts = starts,
        shaft_diam = 0
    );
    
    // Subtract NEMA 17 D-Shaft profile through center
    translate([0, 0, -1])
        linear_extrude(h = worm_l + 2)
            intersection() {
                circle(d = nema_d);
                translate([-nema_d/2, -nema_d/2])
                    square([nema_d, nema_flat]);
            }
}

// =================================================================
// 2. CALCULATE EXACT CENTER DISTANCE
// =================================================================
dist = worm_dist(mod=mod, d=worm_d, starts=starts, teeth=teeth);

// =================================================================
// 3. FULLY ADAPTIVE SKELETONIZED WORM GEAR SECTOR
// =================================================================
translate([dist, 0, 0])
    rotate([90, 0, 0]) // Orient to mesh perpendicular with worm screw
        difference() {
            // --- POSITIVE GEOMETRY ---
            union() {
                // A. Flared Structural Arm
                hull() {
                    cylinder(d = hub_diam, h = hub_thickness, center = true);

                    rotate([0, 0, -sector_angle / 2])
                        rotate_extrude(angle = sector_angle)
                            translate([0, -hub_thickness / 2])
                                square([gear_radius, hub_thickness]);
                }

                // B. Outer Worm Gear Teeth Sector
                intersection() {
                    worm_gear(
                        mod = mod,
                        teeth = teeth,
                        worm_diam = worm_d,
                        worm_starts = starts,
                        shaft_diam = 0
                    );
/*                    
                    rotate([0, 0, -sector_angle / 2])
                        rotate_extrude(angle = sector_angle)
                            translate([0, -hub_thickness / 2])
                                square([gear_radius + 20, hub_thickness]);
*/                    
                }
            }
            
            // --- NEGATIVE GEOMETRY (CUTOUTS) ---
            // C. Center 2020 T-Slot Square Hole
            translate([0, 0, -hub_thickness])
                linear_extrude(h = hub_thickness * 2)
                    square([tslot_size, tslot_size], center = true);
/*

            // D. Fully Adaptive Pocket Matrix
            if (max_pocket_half_angle > 2.0) { // Only cut if sector is wide enough
                for (i = [0 : num_radial_steps - 1]) {
                    // Compute radial bounds for current row
                    p_r1 = r_start + i * (pocket_r_len + radial_rib_thick);
                    p_r2 = p_r1 + pocket_r_len;

                    if (has_dual_pockets) {
                        // Wide Sector: Cut Left and Right pockets around central spine
                        pocket_cutout(p_r1, p_r2, -max_pocket_half_angle, -spine_half_angle, hub_thickness);
                        pocket_cutout(p_r1, p_r2, spine_half_angle, max_pocket_half_angle, hub_thickness);
                    } else {
                        // Narrow Sector: Cut single merged central pocket (no spine)
                        pocket_cutout(p_r1, p_r2, -max_pocket_half_angle, max_pocket_half_angle, hub_thickness);
                    }
                }
            }
            */
        }