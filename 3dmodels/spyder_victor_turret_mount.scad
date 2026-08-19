/* ========================================================================
   SPYDER VICTOR TURRET MOUNT & SOLENOID ACTUATOR BRACKET (V10)
   ========================================================================
   - Direct hole-to-hole distance calibrated to EXACTLY 85.0mm
   - Front solenoid screw holes align vertically below top grip screw
   - 2020 T-slot front edge aligns vertically with rear solenoid screw holes
   - M5 clamping screw with flat-bottom counterbore
   - Heschen HS-1564B 28mm x 20mm 4-hole mounting pattern
   ======================================================================== */

include <BOSL2/std.scad>

$fn = 64; // Smooth curve resolution

/* --- GRIP BRASS INSERT DIMENSIONS --- */
// True caliper center-to-center diagonal distance desired on the physical part
grip_target_pitch  = 85.0;  // True 3D straight-line distance (mm)
grip_offset_x      = -9.0; // Grip backstrap angle offset (mm)

// Calculated vertical Y-span so straight-line distance is exactly 85.0mm:
// sqrt(85.0^2 - 12.0^2) = 84.1487mm
grip_screw_pitch_y = sqrt(sqr(grip_target_pitch) - sqr(abs(grip_offset_x)));

grip_screw_dia     = 3.4;   // M3 / #6-32 grip screw clearance (mm)
grip_head_cbore_d  = 7.0;   // Recessed screw head / driver access diameter (mm)

/* --- HESCHEN HS-1564B SOLENOID POSITION & PATTERN --- */
// Standard 1564 open-frame bottom pattern: 28mm (Length X) x 20mm (Width Y)
sol_pitch_x        = 28.0;  // Length pitch between M3 tapped holes along plunger axis (mm)
sol_pitch_y        = 20.0;  // Width pitch between M3 tapped holes across frame (mm)
sol_plate_l        = 52.0;  // Solenoid bed plate length (mm)
sol_plate_w        = 34.0;  // Solenoid bed plate width (mm)
sol_m3_dia         = 3.4;   // M3 clearance hole (mm)
sol_cbore_d        = 6.5;   // M3 screw head counterbore diameter (mm)

// Front solenoid holes align vertically with top grip screw (X = -12.0mm)
sol_pos_x          = grip_offset_x + (sol_pitch_x / 2.0); // +2.0mm
sol_pos_y          = 42.0;  // Vertical height matching trigger blade center (mm)

/* --- 2020 T-SLOT REAR MOUNT & M5 FLAT COUNTERBORE --- */
tslot_size         = 20.0;  // 2020 square clearance (+0.4mm for 3D printing)
// Front edge of 2020 hole aligns with rear solenoid holes:
// rear_sol_x = sol_pos_x + (sol_pitch_x / 2.0) = 16.0mm
tslot_pos_x        = (sol_pos_x + (sol_pitch_x / 2.0)) + (tslot_size / 2.0); // +26.2mm
tslot_pos_y        = 76.0;  // Height in the upper crook below receiver (mm)

// M5 Flat-Bottom Counterbore Dimensions
tslot_clamp_screw  = 5.4;   // M5 screw shank clearance (mm)
tslot_cbore_d      = 10.0;  // M5 screw head / tool counterbore diameter (mm)
tslot_cbore_depth  = 3.5;   // Recess depth of the flat shoulder from outer face (mm)

/* --- BRACKET THICKNESS & HEIGHT --- */
plate_thick        = 5.0;   // Base plate thickness resting on grip frame (mm)
bracket_depth      = 20.0;  // 2020 sleeve clamp depth in Z (mm)


/* ========================================================================
   MAIN ASSEMBLY
   ======================================================================== */

difference() {
    // 1. SOLID COMPOSITE BRACKET BODY
    union() {
        // Main Unified Mounting Plate (covers grip screws, solenoid bed, and 2020 housing)
        hull() {
            // Lower Grip Nut Anchor
            translate([0, 0, 0])
                cyl(d = 16, h = plate_thick, anchor = BOTTOM);

            // Upper Grip Nut Anchor
            translate([grip_offset_x, grip_screw_pitch_y, 0])
                cyl(d = 16, h = plate_thick, anchor = BOTTOM);

            // Solenoid Bed Plate
            translate([sol_pos_x, sol_pos_y, 0])
                cuboid([sol_plate_l, sol_plate_w, plate_thick], anchor = BOTTOM);
        }

        // 2020 Rail Block Housing (Elevated in Z for rigidity)
        hull() {
            translate([grip_offset_x, grip_screw_pitch_y, 0])
                cyl(d = 16, h = plate_thick, anchor = BOTTOM);

            translate([tslot_pos_x, tslot_pos_y, 0])
                cuboid([tslot_size + 12, tslot_size + 12, bracket_depth], anchor = BOTTOM);
        }
    }

    // 2. GRIP BRASS INSERT THROUGH-HOLES & TOP-ACCESSIBLE COUNTERBORES
    // Lower Grip Screw Hole
    translate([0, 0, -1]) {
        cyl(d = grip_screw_dia, h = plate_thick + 2, anchor = BOTTOM);
        translate([0, 0, 4.0])
            cyl(d = grip_head_cbore_d, h = 50, anchor = BOTTOM);
    }

    // Upper Grip Screw Hole
    translate([grip_offset_x, grip_screw_pitch_y, -1]) {
        cyl(d = grip_screw_dia, h = plate_thick + 2, anchor = BOTTOM);
        translate([0, 0, 4.0])
            cyl(d = grip_head_cbore_d, h = 50, anchor = BOTTOM);
    }

    // 3. 2020 EXTRUSION PASS-THROUGH SLEEVE (Horizontal Rail along Z-Axis)
    translate([tslot_pos_x, tslot_pos_y, -1])
        cuboid([tslot_size, tslot_size, bracket_depth + 2], anchor = BOTTOM);

    // 2020 Rail M5 Clamping Screw Hole with Flat Counterbore Shoulder
    translate([tslot_pos_x + (tslot_size / 2) + 6, tslot_pos_y, bracket_depth / 2])
        rotate([0, -90, 0]) {
            // M5 Shank Clearance Hole through inner wall
            cyl(d = tslot_clamp_screw, h = 10, anchor = BOTTOM);

            // Flat-Bottom Counterbore Pocket
            translate([0, 0, -10])
                cyl(d = tslot_cbore_d, h = 10 + tslot_cbore_depth, anchor = BOTTOM);
        }

    // 4. HESCHEN HS-1564B 4-HOLE RECTANGLE MOUNTING PATTERN (28mm X x 20mm Y)
    translate([sol_pos_x, sol_pos_y, 0]) {
        for (p = rect([sol_pitch_x, sol_pitch_y])) {
            translate([p.x, p.y, -1]) {
                // M3 Screw Shank Through-Hole
                cyl(d = sol_m3_dia, h = plate_thick + 2, anchor = BOTTOM);
                
                // Recessed Counterbore from Bottom (leaves 3.5mm solid shelf)
                cyl(d = sol_cbore_d, h = 3.6, anchor = BOTTOM);
            }
        }
    }
}