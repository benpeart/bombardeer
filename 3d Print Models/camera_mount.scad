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

// --- Lower Arm: HDMI Plug Clearance & Extended Strain Relief ---
// 40mm rigid connector clearance + 28mm strain-relief zone = 68mm total drop
hdmi_clearance     = 68.0; 
ext_w              = 24.0; // Width of the lower drop arm (mm)
cable_dia          = 3.5;  // Semi-circular channel for 3.3mm ultra-slim HDMI wire

// --- Zip-Tie Slot Dimensions ---
ziptie_thick       = 2.4;  // Thickness of slot in X (fits up to 2.2mm zip-tie)
ziptie_width       = 4.5;  // Height of slot in Z (fits up to 4.0mm zip-tie band)
ziptie_x_offset    = 7.5;  // Distance of left/right slots from center cable channel

// --- Vertical Positioning Calculations ---
cam_center_z       = 0.0;  // Centered vertically on the slide-on block
front_y            = sleeve_w / 2;

lower_hole_z       = cam_center_z - (cam_hole_dz / 2); // Z = -6.25mm
ext_top_z          = -sleeve_h / 2;                    // Z = -16.15mm
ext_bottom_z       = lower_hole_z - hdmi_clearance;    // Z = -74.25mm
blend_h            = 14.0;                             // Height of organic transition curve

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
            // 1. Unified 2020 Rail Slide-On Block Housing (Flat Front Face)
            cuboid([sleeve_len, sleeve_w, sleeve_h], rounding = 2, edges = "X");

            // 2. Smooth Curvature Transition (Blends 36mm sleeve down to 24mm arm)
            hull() {
                // Top slice flush with sleeve front face and side profile
                translate([0, front_y - (sleeve_wall / 2), ext_top_z + 1])
                    cuboid([sleeve_len, sleeve_wall, 2], rounding = 1, edges = "Y");

                // Bottom of transition matching the drop arm width
                translate([0, front_y - (sleeve_wall / 2), ext_top_z - blend_h - 1])
                    cuboid([ext_w, sleeve_wall, 2], rounding = 1, edges = "Y");
            }

            // 3. Main Lower Extension Arm (Down to ext_bottom_z)
            translate([0, front_y - (sleeve_wall / 2), (ext_top_z - blend_h + ext_bottom_z) / 2])
                cuboid([ext_w, sleeve_wall, (ext_top_z - blend_h) - ext_bottom_z], rounding = 1, edges = "Y");

            // 4. Underside Gusset Blend (Smooth concave blend under the sleeve block)
            hull() {
                // Anchored to bottom face of sleeve
                translate([0, front_y - sleeve_wall - 3, ext_top_z + 0.5])
                    cube([ext_w, 6, 1], center = true);

                // Slopes down into the back of the extension arm
                translate([0, front_y - sleeve_wall, ext_top_z - 12])
                    cube([ext_w, 0.5, 1], center = true);
            }
        }

        // ================= SUBTRACTIONS =================

        // 1. Full 2020 Rail Bore through X-axis
        cube([sleeve_len + 10, tslot_size, tslot_size], center = true);

        // 2. M5 Rail Clamping Screw Hole (Rear Wall [-Y] into 2020 rear T-slot)
        translate([0, -(tslot_size / 2), 0])
            rotate([90, 0, 0]) {
                // M5 Shank clearance hole through inner rear wall into extrusion
                cyl(d = m5_clamp_screw, h = sleeve_wall + 1, anchor = BOTTOM);

                // Flat-Bottom Counterbore Pocket on rear face
                translate([0, 0, sleeve_wall - m5_cbore_depth])
                    cyl(d = m5_cbore_d, h = m5_cbore_depth + 5, anchor = BOTTOM);
            }

        // 3. M2 Heat-Set Insert Pockets (4x, flush into front wall)
        for (mx = [-cam_hole_dx / 2, cam_hole_dx / 2]) {
            for (mz = [-cam_hole_dz / 2, cam_hole_dz / 2]) {
                translate([mx, front_y, cam_center_z + mz])
                    rotate([-90, 0, 0])
                        cyl(d = m2_insert_dia, h = m2_insert_depth + m2_overdepth, anchor = TOP);
            }
        }

        // 4. Central Window for Rear Camera Flex & SMT Clearance
//        translate([0, front_y, cam_center_z])
//            rotate([90, 0, 0])
//                cuboid([15.0, 16.0, sleeve_wall * 2 + 2], rounding = 1, edges = "Z");

        // 5. Vertical HDMI Cable Channel (Recessed into front face of lower arm)
        translate([0, front_y, (lower_hole_z + ext_bottom_z) / 2])
            rotate([0, 0, 0])
                cyl(d = cable_dia, h = hdmi_clearance + 10, anchor = CENTER);

        // 6. Dual Zip-Tie Slots (Past the 40mm plug drop zone)
        // Positioned at drops of 46mm and 60mm below the lower camera holes
        for (z_drop = [46, 60]) {
            for (x_side = [-ziptie_x_offset, ziptie_x_offset]) {
                translate([x_side, 0, lower_hole_z - z_drop])
                    cube([ziptie_thick, sleeve_w * 2, ziptie_width], center = true);
            }
        }
    }
}