//-----------------------------------------------------------------------
// Yet Another Parameterized Projectbox generator
//
//  This is a box for <template>
//
//  Version 3.0.1 (2024-01-15)
//
// This design is parameterized based on the size of a PCB.
//
// For many/complex cutoutGrills, you might need to adjust
//  the max number of elements in OpenSCAD:
//
//      Preferences->Advanced->Turn off rendering at 250000 elements
//                                                   ^^^^^^
// Fixes: 
//
//-----------------------------------------------------------------------

include <YAPPgenerator_v3.scad>

//---------------------------------------------------------
// This design is parameterized based on the size of a PCB.
//---------------------------------------------------------
// Note: length/lengte refers to X axis, 
//       width/breedte refers to Y axis,
//       height/hoogte refers to Z axis

/*
      padding-back|<------pcb length --->|<padding-front
                            RIGHT
        0    X-axis ---> 
        +----------------------------------------+   ---
        |                                        |    ^
        |                                        |   padding-right 
      Y |                                        |    v
      | |    -5,y +----------------------+       |   ---              
 B    a |         | 0,y              x,y |       |     ^              F
 A    x |         |                      |       |     |              R
 C    i |         |                      |       |     | pcb width    O
 K    s |         |                      |       |     |              N
        |         | 0,0              x,0 |       |     v              T
      ^ |    -5,0 +----------------------+       |   ---
      | |                                        |    padding-left
      0 +----------------------------------------+   ---
        0    X-as --->
                          LEFT
*/


//-- which part(s) do you want to print?
printBaseShell        = true;
printLidShell         = true;
printSwitchExtenders  = false;

// ********************************************************************
// The Following will be used in the pbc array
standoffHeight      = 3.0;  //-- How much the PCB needs to be raised from the base to leave room for solderings and whatnot
standoffDiameter    = 5;
standoffHoleSlack   = 0.5;


//-------------------------------------------------------------------




  
//-- padding between pcb and inside wall
paddingFront        = 40.0;  // Leave space for USB cable
paddingBack         = 0.0;   // behind Arducam HDMI & ESP32 boards
paddingRight        = 3.0;   // along side ESP32 board
paddingLeft         = -5.0;  // along side Raspberry Pi and Arducam HDMI boards

//-- Edit these parameters for your own box dimensions
wallThickness       = 2.0;
basePlaneThickness  = 2.0;
lidPlaneThickness   = 2.0;

//-- Total height of box = lidPlaneThickness 
//                       + lidWallHeight 
//--                     + baseWallHeight 
//                       + basePlaneThickness
//-- space between pcb and lidPlane :=
//--      (baseWallHeight+lidWallHeight) - (standoffHeight+pcbThickness)
baseWallHeight      = 30 + standoffHeight;  // height of tallest PCB + standoffHeight
lidWallHeight       = 5;

//-- ridge where base and lid off box can overlap
//-- Make sure this isn't less than lidWallHeight
ridgeHeight         = 5.0;
ridgeSlack          = 0.2;
roundRadius         = 3.0;


// Set the layer height of your printer
printerLayerHeight  = 0.2;





//---------------------------
//--     C O N T R O L     --
//---------------------------
// -- Render --
renderQuality             = 8;          //-> from 1 to 32, Default = 8

// --Preview --
previewQuality            = 5;          //-> from 1 to 32, Default = 5
showSideBySide            = true;       //-> Default = true
onLidGap                  = 0;  // tip don't override to animate the lid opening
colorLid                  = "YellowGreen";   
alphaLid                  = 1;
colorBase                 = "BurlyWood";
alphaBase                 = 1;
hideLidWalls              = false;      //-> Remove the walls from the lid : only if preview and showSideBySide=true 
hideBaseWalls             = false;      //-> Remove the walls from the base : only if preview and showSideBySide=true  
showOrientation           = true;       //-> Show the Front/Back/Left/Right labels : only in preview
showPCB                   = false;       //-> Show the PCB in red : only in preview 
showSwitches              = false;      //-> Show the switches (for pushbuttons) : only in preview 
showButtonsDepressed      = false;      //-> Should the buttons in the Lid On view be in the pressed position
showOriginCoordBox        = false;      //-> Shows red bars representing the origin for yappCoordBox : only in preview 
showOriginCoordBoxInside  = false;      //-> Shows blue bars representing the origin for yappCoordBoxInside : only in preview 
showOriginCoordPCB        = false;      //-> Shows blue bars representing the origin for yappCoordBoxInside : only in preview 
showMarkersPCB            = false;      //-> Shows black bars corners of the PCB : only in preview 
showMarkersCenter         = false;      //-> Shows magenta bars along the centers of all faces  
inspectX                  = 0;          //-> 0=none (>0 from Back)
inspectY                  = 0;          //-> 0=none (>0 from Right)
inspectZ                  = 0;          //-> 0=none (>0 from Bottom)
inspectXfromBack          = true;       //-> View from the inspection cut foreward
inspectYfromLeft          = true;       //-> View from the inspection cut to the right
inspectZfromBottom        = true;       //-> View from the inspection cut up
//---------------------------
//--     C O N T R O L     --
//---------------------------

//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Start of Debugging config (used if not overridden in template)
// ------------------------------------------------------------------
// ------------------------------------------------------------------

//==================================================================
//  *** Shapes ***
//------------------------------------------------------------------
//  There are a view pre defines shapes and masks
//  shapes:
//      shapeIsoTriangle, shapeHexagon, shape6ptStar
//
//  masks:
//      maskHoneycomb, maskHexCircles, maskBars, maskOffsetBars
//
//------------------------------------------------------------------
// Shapes should be defined to fit into a 1x1 box (+/-0.5 in X and Y) - they will 
// be scaled as needed.
// defined as a vector of [x,y] vertices pairs.(min 3 vertices)
// for example a triangle could be [yappPolygonDef,[[-0.5,-0.5],[0,0.5],[0.5,-0.5]]];
// To see how to add your own shapes and mask see the YAPPgenerator program
//------------------------------------------------------------------


// Show sample of a Mask
//SampleMask(maskHoneycomb);

//===================================================================
// *** PCBs ***
// Printed Circuit Boards
//-------------------------------------------------------------------
//  Default origin =  yappCoordPCB : yappCoordBoxInside[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = name
//    p(1) = length
//    p(2) = width
//    p(3) = posx
//    p(4) = posy
//    p(5) = Thickness
//    p(6) = standoffHeight
//    p(7) = standoffDiameter
//    p(8) = standoffPinDiameter
//    p(9) = standoffHoleSlack (default to 0.4)
//   Optional:

//The following can be used to get PCB values elsewhere in the script - not in pcb definition. 
//If "PCB Name" is omitted then "Main" is used
//  pcbLength           --> pcbLength("PCB Name")
//  pcbWidth            --> pcbWidth("PCB Name")
//  pcbThickness        --> pcbThickness("PCB Name") 
//  standoffHeight      --> standoffHeight("PCB Name") 
//  standoffDiameter    --> standoffDiameter("PCB Name") 
//  standoffPinDiameter --> standoffPinDiameter("PCB Name") 
//  standoffHoleSlack   --> standoffHoleSlack("PCB Name") 

pcb = 
[
  // 1. ESP32 Controller (130 x 36 mm, thickness = 1.7mm, front edge at X = 136.0mm)
  ["ESP32",       130.0, 36.0,  6.0, 72.0, 1.7, standoffHeight, standoffDiameter, 3.0 - standoffHoleSlack, standoffHoleSlack]

  // 2. Raspberry Pi 5 (85 x 56 mm, thickness = 1.6mm)
 ,["Raspberry Pi", 85.0, 56.0, 51.0,  8.0, 1.6, standoffHeight, standoffDiameter + 1.0, 2.5, standoffHoleSlack]

  // 3. Arducam CSI-to-HDMI Board (27.6 x 25 mm, thickness = 1.2mm)
 ,["HDMI Adapter", 27.6, 25.0,  6.0,  8.0, 1.2, standoffHeight, standoffDiameter, 2.0 - standoffHoleSlack, standoffHoleSlack]

  // 4. MOSFET (32.85 x 16.58 mm, thickness = 1.5mm)
 ,["MOSFET", 32.85, 16.58,  15.0,  47.5, 1.5, standoffHeight, standoffDiameter, 2.0, standoffHoleSlack]
];


//===================================================================
// *** PCB Supports ***
// Pin and Socket standoffs 
//-------------------------------------------------------------------
//  Default origin =  yappCoordPCB : pcb[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = posx
//    p(1) = posy
//   Optional:
//    p(2) = Height to bottom of PCB : Default = standoff_Height
//    p(3) = PCB Gap : Default = -1 : Default for yappCoordPCB=pcb_Thickness, yappCoordBox=0
//    p(4) = standoff_Diameter    Default = standoff_Diameter;
//    p(5) = standoff_PinDiameter Default = standoff_PinDiameter;
//    p(6) = standoff_HoleSlack   Default = standoff_HoleSlack;
//    p(7) = filletRadius (0 = auto size)
//    n(a) = { <yappBoth> | yappLidOnly | yappBaseOnly }
//    n(b) = { <yappPin>, yappHole } // Baseplate support treatment
//    n(c) = { yappAllCorners, yappFrontLeft | <yappBackLeft> | yappFrontRight | yappBackRight }
//    n(d) = { <yappCoordPCB> | yappCoordBox | yappCoordBoxInside }
//    n(e) = { yappNoFillet }
//    n(f) = [yappPCBName, "XXX"] : {Specify a PCB defaults to "Main"
//-------------------------------------------------------------------
pcbStands = 
[
  // --- SET 1: ESP32 Board (Bottom pin + Lid clamping socket) ---
  [2.78,   2.59,  yappBoth, yappPin, [yappPCBName, "ESP32"]]
 ,[126.61, 2.72,  yappBoth, yappPin, [yappPCBName, "ESP32"]]
 ,[2.78,   33.32, yappBoth, yappPin, [yappPCBName, "ESP32"]]
 ,[126.61, 33.20, yappBoth, yappPin, [yappPCBName, "ESP32"]]
 
 // --- SET 2: Raspberry Pi 5 (Solid base standoffs, 5.0mm high, NO lid stands) ---
// ,[3.5,        3.5,        yappBaseOnly, yappHole, [yappPCBName, "Raspberry Pi"]]
// ,[3.5 + 58.0, 3.5,        yappBaseOnly, yappHole, [yappPCBName, "Raspberry Pi"]]
// ,[3.5,        3.5 + 49.0, yappBaseOnly, yappHole, [yappPCBName, "Raspberry Pi"]]
// ,[3.5 + 58.0, 3.5 + 49.0, yappBaseOnly, yappHole, [yappPCBName, "Raspberry Pi"]]
 
  // --- SET 3: Arducam HDMI Adapter (Bottom pin + Lid clamping socket) ---
 ,[9.0,        2.5,        yappBoth, yappPin, [yappPCBName, "HDMI Adapter"]]
 ,[9.0 + 12.3, 2.5,        yappBoth, yappPin, [yappPCBName, "HDMI Adapter"]]
 ,[9.0,        2.5 + 20.6, yappBoth, yappPin, [yappPCBName, "HDMI Adapter"]]
 ,[9.0 + 12.3, 2.5 + 20.6, yappBoth, yappPin, [yappPCBName, "HDMI Adapter"]]

  // --- SET 4: MOSFET (Bottom pin + Lid clamping socket) ---
 ,[30.5, 2.5,        yappBaseOnly, yappHole, yappSelfThreading, [yappPCBName, "MOSFET"]]
 ,[10.5, 2.5 + 5.75, yappBaseOnly, yappHole, yappSelfThreading, [yappPCBName, "MOSFET"]]
 ,[30.5, 2.5 + 11.5, yappBaseOnly, yappHole, yappSelfThreading, [yappPCBName, "MOSFET"]]

];


//===================================================================
//  *** Connectors ***
//  Standoffs with hole through base and socket in lid for screw type connections.
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//  
//  Parameters:
//   Required:
//    p(0) = posx
//    p(1) = posy
//    p(2) = pcbStandHeight
//    p(3) = screwDiameter
//    p(4) = screwHeadDiameter (don't forget to add extra for the fillet)
//    p(5) = insertDiameter
//    p(6) = outsideDiameter
//   Optional:
//    p(7) = PCB Gap : Default = -1 : Default for yappCoordPCB=pcbThickness, yappCoordBox=0
//    p(8) = filletRadius : Default = 0/Auto(0 = auto size)
//    n(a) = { <yappAllCorners>, yappFrontLeft | yappFrontRight | yappBackLeft | yappBackRight }
//    n(b) = { <yappCoordBox> | yappCoordPCB |  yappCoordBoxInside }
//    n(c) = { yappNoFillet }
//    n(d) = [yappPCBName, "XXX"] : {XXX = the PCB name: Default "Main"}
//-------------------------------------------------------------------
connectors   =
[
//  [9, 15, standoffHeight("PCB3"), 2.5, 6 + 1.25, 4.0, 9, yappAllCorners, [yappPCBName, "PCB3"]]
];


//===================================================================
//  *** Cutouts ***
//    There are 6 cutouts one for each surface:
//      cutoutsBase (Bottom), cutoutsLid (Top), cutoutsFront, cutoutsBack, cutoutsLeft, cutoutsRight
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//
//                        Required                Not Used        Note
//----------------------+-----------------------+---------------+------------------------------------
//  yappRectangle       | width, length         | radius        |
//  yappCircle          | radius                | width, length |
//  yappRoundedRect     | width, length, radius |               |     
//  yappCircleWithFlats | width, radius         | length        | length=distance between flats
//  yappCircleWithKey   | width, length, radius |               | width = key width length=key depth
//  yappPolygon         | width, length         | radius        | yappPolygonDef object must be
//                      |                       |               | provided
//----------------------+-----------------------+---------------+------------------------------------
//
//  Parameters:
//   Required:
//    p(0) = from Back
//    p(1) = from Left
//    p(2) = width
//    p(3) = length
//    p(4) = radius
//    p(5) = shape : { yappRectangle | yappCircle | yappPolygon | yappRoundedRect 
//                     | yappCircleWithFlats | yappCircleWithKey }
//  Optional:
//    p(6) = depth : Default = 0/Auto : 0 = Auto (plane thickness)
//    p(7) = angle : Default = 0
//    n(a) = { yappPolygonDef } : Required if shape = yappPolygon specified -
//    n(b) = { yappMaskDef } : If a yappMaskDef object is added it will be used as a mask 
//                             for the cutout.
//    n(c) = { [yappMaskDef, hOffset, vOffset, rotation] } : If a list for a mask is added 
//                              it will be used as a mask for the cutout. With the Rotation 
//                              and offsets applied. This can be used to fine tune the mask
//                              placement within the opening.
//    n(d) = { <yappCoordPCB> | yappCoordBox | yappCoordBoxInside }
//    n(e) = { <yappOrigin>, yappCenter }
//    n(f) = { <yappGlobalOrigin>, yappLeftOrigin } // Only affects Top(lid), Back and Right Faces
//    n(g) = [yappPCBName, "XXX"] : {Specify a PCB defaults to "Main"
//-------------------------------------------------------------------
cutoutsBase = 
[
  // Base Floor Honeycomb: Positioned directly under the Pi 5's active cooler intake
  [pcbLength("Raspberry Pi")/2, pcbWidth("Raspberry Pi")/2, 50.0, 42.0, 5, yappPolygon, 0, 0, yappCenter, shapeHexagon, [maskHoneycomb, 0, 1.5, 0], [yappPCBName, "Raspberry Pi"]]
];

cutoutsLid  = 
[
];

cutoutsFront =  
[
];

cutoutsBack = 
[
  // HDMI Port: Centered on HDMI adapter board width
  [pcbWidth("HDMI Adapter")/2, pcbThickness("HDMI Adapter") + 2.6, 21.0, 9.0, 1.5, yappRoundedRect, yappCenter, [yappPCBName, "HDMI Adapter"]]

  // Stepper Motor Wires: Centered relative to the ESP32 board
 ,[pcbWidth("ESP32")/2, pcbThickness("ESP32") + 14.0, 30.0, 10.0, 3.0, yappRoundedRect, yappCenter, [yappPCBName, "ESP32"]]
];

cutoutsLeft =   
[
  // Long Wall on Left (Y = 0): Raspberry Pi 5 USB-C Power In
  // Port center at 11.2mm along the Pi's length
  [11.2, pcbThickness("Raspberry Pi"), 13.0, 8.0, 2.0, yappRoundedRect, yappCenter, [yappPCBName, "Raspberry Pi"]]

];

cutoutsRight =  
[
];



//===================================================================
//  *** Snap Joins ***
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = posx | posy
//    p(1) = width
//    p(2) = { yappLeft | yappRight | yappFront | yappBack } : one or more
//   Optional:
//    n(a) = { <yappOrigin>, yappCenter }
//    n(b) = { yappSymmetric }
//    n(c) = { yappRectangle } == Make a diamond shape snap
//-------------------------------------------------------------------
snapJoins   =   
[
    [30, 10, yappFront, yappCenter, yappSymmetric]
   ,[30, 10, yappBack,  yappCenter, yappSymmetric]
   ,[45, 10, yappRight, yappCenter, yappSymmetric]
   ,[45, 10, yappLeft,  yappCenter, yappSymmetric]

];

//===================================================================
//  *** Box Mounts ***
//    Mounting tabs on the outside of the box
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = pos : position along the wall : [pos,offset] : vector for position and offset X.
//                    Position is to center of mounting screw in leftmost position in slot
//    p(1) = screwDiameter
//    p(2) = width of opening in addition to screw diameter 
//                    (0=Circular hole screwWidth = hole twice as wide as it is tall)
//    p(3) = height
//   Optional:
//    p(4) = filletRadius : Default = 0/Auto(0 = auto size)
//    n(a) = { yappLeft | yappRight | yappFront | yappBack } : one or more
//    n(b) = { yappNoFillet }
//    n(c) = { <yappBase>, yappLid }
//    n(d) = { yappCenter } : shifts Position to be in the center of the opening instead of 
//                            the left of the opening
//    n(e) = { <yappGlobalOrigin>, yappLeftOrigin } : Only affects Back and Right Faces
//-------------------------------------------------------------------
boxMounts =
[
];

//===================================================================
//  *** Light Tubes ***
//-------------------------------------------------------------------
//  Default origin = yappCoordPCB: PCB[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = posx
//    p(1) = posy
//    p(2) = tubeLength
//    p(3) = tubeWidth
//    p(4) = tubeWall
//    p(5) = gapAbovePcb
//    p(6) = { yappCircle | yappRectangle } : tubeType    
//   Optional:
//    p(7) = lensThickness (how much to leave on the top of the lid for the 
//           light to shine through 0 for open hole : Default = 0/Open
//    p(8) = Height to top of PCB : Default = standoffHeight+pcbThickness
//    p(9) = filletRadius : Default = 0/Auto 
//    n(a) = { <yappCoordPCB> | yappCoordBox | yappCoordBoxInside } 
//    n(b) = { <yappGlobalOrigin>, yappLeftOrigin }
//    n(c) = { yappNoFillet }
//    n(d) = [yappPCBName, "XXX"] : {Specify a PCB defaults to "Main"
//-------------------------------------------------------------------
lightTubes =
[
];

//===================================================================
//  *** Push Buttons ***
//-------------------------------------------------------------------
//  Default origin = yappCoordPCB: PCB[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = posx
//    p(1) = posy
//    p(2) = capLength 
//    p(3) = capWidth 
//    p(4) = capRadius 
//    p(5) = capAboveLid
//    p(6) = switchHeight
//    p(7) = switchTravel
//    p(8) = poleDiameter
//   Optional:
//    p(9) = Height to top of PCB : Default = standoffHeight + pcbThickness
//    p(10) = { yappRectangle | yappCircle | yappPolygon | yappRoundedRect 
//                    | yappCircleWithFlats | yappCircleWithKey } : Shape, Default = yappRectangle
//    p(11) = angle : Default = 0
//    p(12) = filletRadius          : Default = 0/Auto 
//    p(13) = buttonWall            : Default = 2.0;
//    p(14) = buttonPlateThickness  : Default= 2.5;
//    p(15) = buttonSlack           : Default= 0.25;
//    n(a) = { <yappCoordPCB> | yappCoordBox | yappCoordBoxInside } 
//    n(b) = { <yappGlobalOrigin>,  yappLeftOrigin }
//    n(c) = { yappNoFillet }
//    n(d) = [yappPCBName, "XXX"] : {Specify a PCB defaults to "Main"
//-------------------------------------------------------------------
pushButtons = 
[
];
             
//===================================================================
//  *** Labels ***
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//
//  Parameters:
//   p(0) = posx
//   p(1) = posy/z
//   p(2) = rotation degrees CCW
//   p(3) = depth : positive values go into case (Remove) negative valies are raised (Add)
//   p(4) = { yappLeft | yappRight | yappFront | yappBack | yappLid | yappBaseyappLid } : plane
//   p(5) = font
//   p(6) = size
//   p(7) = "label text"
//  Optional:
//   p(8) = Expand : Default = 0 : mm to expand text by (making it bolder) 
//-------------------------------------------------------------------
labelsPlane =
[
    [5, 5, 0, 1, yappLid, "Liberation Mono:style=bold", 5, "Bombardeer" ]
];


//===================================================================
//  *** Ridge Extension ***
//    Extension from the lid into the case for adding split opening at various heights
//-------------------------------------------------------------------
//  Default origin = yappCoordBox: box[0,0,0]
//
//  Parameters:
//   Required:
//    p(0) = pos
//    p(1) = width
//    p(2) = height : Where to relocate the seam : yappCoordPCB = Above (positive) the PCB
//                                                yappCoordBox = Above (positive) the bottom of the shell (outside)
//   Optional:
//    n(a) = { <yappOrigin>, yappCenter } 
//    n(b) = { <yappCoordPCB> | yappCoordBox | yappCoordBoxInside }
//    n(c) = { yappLeftOrigin, <yappGlobalOrigin> } // Only affects Top(lid), Back and Right Faces
//    n(d) = [yappPCBName, "XXX"] : {Specify a PCB defaults to "Main"
//
// Note: Snaps should not be placed on ridge extensions as they remove the ridge to place them.
//-------------------------------------------------------------------
ridgeExtLeft =
[
];

ridgeExtRight =
[
];

ridgeExtFront =
[
];

ridgeExtBack =
[
];



//========= HOOK functions ============================
  
// Hook functions allow you to add 3d objects to the case.
// Lid/Base = Shell part to attach the object to.
// Inside/Outside = Join the object from the midpoint of the shell to the inside/outside.
// Pre = Attach the object Pre before doing Cutouts/Stands/Connectors. 


//===========================================================
// origin = box(0,0,0)
module hookLidInside()
{
  //if (printMessages) echo("hookLidInside() ..");
  
} // hookLidInside()
  

//===========================================================
// origin = box(0,0,shellHeight)
module hookLidOutside()
{
  //if (printMessages) echo("hookLidOutside() ..");
  
} // hookLidOutside()

//===========================================================
//===========================================================
// origin = box(0,0,0)
module hookBaseInside()
{
  //if (printMessages) echo("hookBaseInside() ..");
  
} // hookBaseInside()

//===========================================================
// origin = box(0,0,0)
module hookBaseOutside()
{
  //if (printMessages) echo("hookBaseOutside() ..");
  
} // hookBaseInside()

// **********************************************************
// **********************************************************
// **********************************************************
// *************** END OF TEMPLATE SECTION ******************
// **********************************************************
// **********************************************************
// **********************************************************

//---- This is where the magic happens ----

//===================================================================
// *** Raspberry Pi 5 Standoff with Bottom Screw Head Counterbore ***
//===================================================================
module rpi_boss_post(x, y)
{
  fillet_r = 1.5;
  stand_d  = standoffDiameter("Raspberry Pi");
  stand_h  = standoffHeight("Raspberry Pi");

  translate([x, y, basePlaneThickness])
  {
    // 1. Standoff post
    cylinder(d = stand_d, h = stand_h, $fn = 48);

    // 2. Matching concave circular fillet to floor plane
    rotate_extrude($fn = 48)
    {
      translate([stand_d / 2, 0, 0])
      {
        difference()
        {
          square([fillet_r, fillet_r]);
          translate([fillet_r, fillet_r, 0])
            circle(r = fillet_r, $fn = 32);
        }
      }
    }
  }
}

module rpi_bottom_screw_cuts(x, y)
{
  hole_d       = standoffPinDiameter("Raspberry Pi");
  head_d       = 5.0; // Recessed head clearance (socket or pan head)
  head_depth   = 2.5; // Recessed depth into base plane
  total_height = basePlaneThickness + 5.0 + 1.0;

  translate([x, y, 0])
  {
    // Full through-hole through base and standoff
    translate([0, 0, -0.5])
      cylinder(d = hole_d, h = total_height, $fn = 32);

    // Recessed screw head pocket from bottom surface (Z = 0)
    translate([0, 0, -0.1])
      cylinder(d = head_d, h = head_depth + 0.1, $fn = 36);
  }
}

// Helper lookups for PCB positions from the pcb array
function pcbPosX(name="Main") = [for (p = pcb) if (p[0] == name) p[3]][0];
function pcbPosY(name="Main") = [for (p = pcb) if (p[0] == name) p[4]][0];

// Raspberry Pi 5 HAT standard mounting hole offsets
rpi_hole_margin = 3.5;
rpi_hole_dist_x = 58.0;
rpi_hole_dist_y = 49.0;

// Base origin offsets in world coordinates
rpi_base_x = wallThickness + paddingBack + pcbPosX("Raspberry Pi") + rpi_hole_margin;
rpi_base_y = wallThickness + paddingLeft + pcbPosY("Raspberry Pi") + rpi_hole_margin;

rpi_holes = 
[
  [rpi_base_x,                   rpi_base_y],
  [rpi_base_x + rpi_hole_dist_x, rpi_base_y],
  [rpi_base_x,                   rpi_base_y + rpi_hole_dist_y],
  [rpi_base_x + rpi_hole_dist_x, rpi_base_y + rpi_hole_dist_y]
];

//---- Build Enclosure with Counterbored Through-Standoffs ----
difference()
{
  union()
  {
    YAPPgenerate();

    if (printBaseShell)
    {
      color(colorBase)
      {
        for (h = rpi_holes)
          rpi_boss_post(h[0], h[1]);
      }
    }
  }

  // Bore the recessed screw head pockets and through-holes
  if (printBaseShell)
  {
    for (h = rpi_holes)
      rpi_bottom_screw_cuts(h[0], h[1]);
  }
}