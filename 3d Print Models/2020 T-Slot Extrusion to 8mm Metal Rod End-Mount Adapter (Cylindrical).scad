// ====================================================================
// 2020 T-Slot Extrusion to 8mm Metal Rod End-Mount Adapter (Cylindrical)
// ====================================================================
$fn = 64; // High resolution for smooth holes

// --- Parametric Dimensions ---
tslot_size             = 20.0; // Standard 2020 profile width (20mm)
tslot_clearance        = 0.0;  // Print tolerance for 2020 fit (+0.2mm)
socket_depth           = 12.0; // Depth of sleeve wrapping around 2020 end (20mm)

rod_diam               = 8.0;  // Metal rod diameter (8mm)
rod_clearance          = 0.1;  // Print tolerance for rod fit (+0.2mm)
rod_socket_depth       = 15.0; // Insertion depth for the 8mm rod (25mm)

wall_thickness         = 3.0;  // Wall thickness at thinnest point (3.5mm)
partition_thick        = 0.0;  // Divider thickness between 2020 end and rod (5.0mm)


// --- M4 Rod Set-Screw & Countersink Parameters ---
m4_rod_tap_diam        = 3.5;  // M4 tap hole diameter for threading into plastic (3.5mm)
m4_cs_diam             = 8.5;  // Recess diameter for M4 screw head (8.5mm)
m4_cs_depth            = 4.0;  // Countersink depth from outer cylinder surface (6.0mm)

// --- M5 - Clamps 2020 T-Slot & Countersink Parameters ---
m5_bolt_diam           = 5.1;  // Hole diameter for M5 bolt
m5_cs_diam             = 9.5;  // Recess diameter for M4 screw head (9.5mm)
m5_cs_depth            = 4.0;  // Countersink depth from outer cylinder surface (6.0mm)

// --- Derived Dimensions ---
tslot_width  = tslot_size + (tslot_clearance * 2);
outer_diam   = sqrt(2 * pow(tslot_width, 2)) + (wall_thickness * 2); 
total_length = socket_depth + partition_thick + rod_socket_depth;

// --- Main Part Generation ---
difference() {
    // 1. Solid Outer Cylinder Shell
    cylinder(d = outer_diam, h = total_length, center = true);

    // 2. 2020 Extrusion Socket (Bottom)
    translate([0, 0, -total_length/2 + socket_depth/2 - 0.1])
        cube([
            tslot_width, 
            tslot_width, 
            socket_depth + 0.2
        ], center = true);

    // 3. 8mm Rod Socket (Top)
    translate([0, 0, total_length/2 - rod_socket_depth/2 + 0.1])
        cylinder(
            d = rod_diam + (rod_clearance * 2), 
            h = rod_socket_depth + 0.2, 
            center = true
        );

    // 4. Center M5 Mounting Hole (Secures adapter into 2020 end-hole)
//    cylinder(d = m5_bolt_diam, h = total_length + 2, center = true);

    // 5. Top M4 Rod Set-Screw (Single-sided: Center to outer edge only)
    translate([0, 0, total_length/2 - (rod_socket_depth / 2)]) {
        rotate([0, 90, 0]) {
            // M4 Tap Hole extending from center axis out to outer wall only
            cylinder(d = m4_rod_tap_diam, h = outer_diam/2 + 1, center = false);

            // Entry-side Countersink/Counterbore for M4 screw head
            translate([0, 0, outer_diam/2 - m4_cs_depth])
                cylinder(d = m4_cs_diam, h = m4_cs_depth + 5);
        }
    }

    // 6. Bottom Side Clamping Set-Screw Hole (M5 - Clamps 2020 T-Slot)
    translate([0, 0, -total_length/2 + (socket_depth / 2)])
        rotate([0, 90, 0]) {
            // M5 Hole extending from center axis out to outer wall only
            cylinder(d = m5_bolt_diam, h = outer_diam/2 + 2, center = false);

            // Entry-side Countersink/Counterbore for M5 screw head
            translate([0, 0, outer_diam/2 - m5_cs_depth])
                cylinder(d = m5_cs_diam, h = m5_cs_depth + 5);
        }
}