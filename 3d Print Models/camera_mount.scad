include <BOSL2/std.scad>

$fn = 64;

// =========================================================================
// USER PARAMETERS & DIMENSIONS
// =========================================================================

// --- 2020 Extrusion Sleeve (Slide-On Block) ---
tslot_size         = 20.3; // 2020 extrusion width (+0.3mm clearance for 3D printing)
sleeve_wall        = 6.0;  // Solid perimeter wall thickness around extrusion
sleeve_len         = 36.0; // Length along the 2020 rail (X-axis)
sleeve_w           = tslot_size + (2 * sleeve_wall); // ~32.3mm (Y-axis)
sleeve_h           = tslot_size + (2 * sleeve_wall); // ~32.3mm (Z-axis)

// --- M5 Clamping Screw (Flat-Bottom Counterbore on -Y Rear Face) ---
m5_clamp_screw     = 5.5;  // M5 shank clearance diameter
m5_cbore_d         = 10.0; // Counterbore diameter for M5 socket head & tool
m5_cbore_depth     = 4.5;  // Counterbore shoulder pocket depth

// --- Camera Module 3 & Adapter Mount Hole Spacing (Facing +Y) ---
cam_hole_dx        = 21.0; // Horizontal spacing (X-axis)
cam_hole_dz        = 13.0; // Vertical spacing (Z-axis)

// --- M2 Heat-Set Inserts (Flush into Block Wall) ---
m2_insert_dia      = 2.7;  // Heat-set insert outer diameter
m2_insert_depth    = 4.0;  // Insert pocket depth
m2_overdepth       = 1.0;  // Clearance reservoir for displaced molten plastic

// --- Lower Arm: Shifted Drop Arm & Cable Channel ---
hdmi_clearance     = 68.0;
ext_w              = 24.0; // Width of the lower drop arm (mm along X)
y_shift            = 5.0;  // Shift arm forward by 5mm in +Y
cable_dia          = 3.5;  // Semi-circular channel for 3.3mm ultra-slim HDMI wire

// --- Zip-Tie Slot Dimensions ---
ziptie_thick       = 2.4;  // Thickness of slot in X (fits up to 2.2mm zip-tie)
ziptie_width       = 4.5;  // Height of slot in Z (fits up to 4.0mm zip-tie band)
ziptie_x_offset    = 7.5;  // Distance of left/right slots from center cable channel

// --- Vertical & Positional Calculations ---
cam_center_z       = 0.0;
front_y            = sleeve_w / 2;                           // Sleeve block front face (~16.15mm)
rear_y             = -sleeve_w / 2;                          // Sleeve block rear face (~ -16.15mm)
sleeve_wall_center = front_y - (sleeve_wall / 2);            // Center of original 6mm front sleeve wall
arm_front_y        = front_y + y_shift;                      // Shifted front face (~21.15mm)
arm_center_y       = sleeve_wall_center + y_shift;           // Shifted arm center in Y

lower_hole_z       = cam_center_z - (cam_hole_dz / 2); // Z = -6.5mm
ext_top_z          = -sleeve_h / 2;                    // Z = -16.15mm
ext_bottom_z       = lower_hole_z - hdmi_clearance;    // Z = -74.5mm
blend_h            = 14.0;                             // Height of transition curve
gusset_depth_y     = 6.0;                              // Rear anchor depth into sleeve base

// =========================================================================
// RENDER
// =========================================================================

camera_turret_mount();

// =========================================================================
// MODULE DEFINITION
// =========================================================================

module camera_turret_mount() {
    difference() {
        union() {
            // 1. Unified 2020 Rail Slide-On Block Housing
            cuboid([sleeve_len, sleeve_w, sleeve_h], rounding = 2, edges = "X");

            // 2. Continuous Arm & Gusset Transition
            hull() {
                // Top profile at base of sleeve: full front wall + underside gusset anchor
                translate([0, sleeve_wall_center - (gusset_depth_y / 2), ext_top_z + 1])
                    cuboid([sleeve_len, sleeve_wall + gusset_depth_y, 2], rounding = 1, edges = "Y");

                // Bottom profile at end of transition matching the shifted 24x6mm drop arm
                translate([0, arm_center_y, ext_top_z - blend_h - 1])
                    cuboid([ext_w, sleeve_wall, 2], rounding = 1, edges = "Y");
            }

            // 3. Main Lower Extension Arm (Shifted +5mm in Y, 6mm thickness preserved)
            translate([0, arm_center_y, (ext_top_z - blend_h + ext_bottom_z) / 2])
                cuboid([ext_w, sleeve_wall, (ext_top_z - blend_h) - ext_bottom_z], rounding = 1, edges = "Y");
        }

        // ================= SUBTRACTIONS =================

        // 1. Full 2020 Rail Bore through X-axis
        cube([sleeve_len + 10, tslot_size, tslot_size], center = true);

        // 2. M5 Rail Clamping Screw Hole (Rear Wall [-Y] into 2020 rear T-slot)
        // Passes completely from the outer rear face through into the inner extrusion slot
        translate([0, -(tslot_size / 2) + 2, 0])
            rotate([90, 0, 0]) {
                // Shank clearance hole extending cleanly into the inner cavity
                cyl(d = m5_clamp_screw, h = sleeve_wall + 4, anchor = BOTTOM);

                // Counterbore pocket positioned from the outer rear face
                translate([0, 0, sleeve_wall + 2 - m5_cbore_depth])
                    cyl(d = m5_cbore_d, h = m5_cbore_depth + 5, anchor = BOTTOM);
            }

        // 3. M2 Heat-Set Insert Pockets (4x, flush into front wall of sleeve)
        for (mx = [-cam_hole_dx / 2, cam_hole_dx / 2]) {
            for (mz = [-cam_hole_dz / 2, cam_hole_dz / 2]) {
                translate([mx, front_y, cam_center_z + mz])
                    rotate([-90, 0, 0])
                        cyl(d = m2_insert_dia, h = m2_insert_depth + m2_overdepth, anchor = TOP);
            }
        }

        // 4. Vertical HDMI Cable Channel (Shifted to the new +5mm front face)
        translate([0, arm_front_y, (lower_hole_z + ext_bottom_z) / 2])
            rotate([0, 0, 0])
                cyl(d = cable_dia, h = hdmi_clearance + 10, anchor = CENTER);

        // 5. Dual Zip-Tie Slots (Centered through the shifted 6mm wall)
        for (z_drop = [46, 60]) {
            for (x_side = [-ziptie_x_offset, ziptie_x_offset]) {
                translate([x_side, arm_center_y, lower_hole_z - z_drop])
                    cube([ziptie_thick, sleeve_wall + 4, ziptie_width], center = true);
            }
        }
    }
}