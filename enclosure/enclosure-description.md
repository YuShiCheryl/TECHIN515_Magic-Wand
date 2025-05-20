# Description of Materials, Design Decisions, and Battery

## Materials

For the magic wand enclosure, I selected materials that balance functionality, durability, and aesthetic appeal:

- **Chipboard**: Used for the main body and wings, providing a lightweight yet sturdy structure with better durability than standard cardboard.
- **Wooden Dowel**: Selected for the handle/stem of the wand, offering a comfortable grip and natural feel.
- **Adhesive**: Super glue (cyanoacrylate) to secure all components together, providing strong, permanent bonds.
- **PCB Mount**: Custom-designed internal mounting system to secure the electronics.
- **LED Indicator**: Small red LED visible through an opening to indicate operational status.

## Design Decisions

### Winged Magic Wand Concept

I chose a design inspired by magical elements from fantasy stories, incorporating wings to evoke a magical "Golden Snitch" or flying wand aesthetic. This approach has several advantages:

1. **Unique Visual Identity**: The winged design creates an instantly recognizable silhouette that distinguishes this wand from conventional designs.
   
2. **Practical Component Housing**: The rectangular box section provides an ideal housing for the electronic components, offering ample space for the circuit board, MPU6050 sensor, and button.

3. **Sensor Orientation**: The box design allows for consistent orientation of the MPU6050 sensor, which is critical for reliable gesture recognition.

4. **Intuitive Ergonomics**: The wooden dowel handle provides a natural grip point, guiding users to hold the wand correctly for optimal sensor readings.

5. **Access to Components**: The design allows for a removable panel to access internal components for maintenance or battery replacement.

### Dimensions and Proportions

The enclosure dimensions were carefully considered:

- **Main Body**: Approximately 8cm x 4cm x 3cm box to house the electronic components
- **Wings**: Extend 5cm on each side, designed with a curved profile for aesthetic appeal
- **Handle**: 25cm long wooden dowel, providing comfortable grip and sufficient length for performing gestures
- **Total Length**: Approximately 33cm from tip to end of handle

These proportions were determined after testing various prototypes to ensure comfortable handling during gesture performance while maintaining a balanced weight distribution.

### Component Placement

The internal components are arranged to optimize performance:

- **Circuit Board**: Positioned in the center of the box for stability
- **MPU6050 Sensor**: Mounted firmly to prevent unwanted movement, oriented with X-axis aligned with the length of the wand
- **Button**: Accessible on top of the box for easy triggering of gesture recognition
- **LED Indicator**: Visible from the top to provide visual feedback about operation status
- **Battery**: Secured at the bottom of the main body to lower the center of gravity

## Battery Selection and Power Management

### Battery Specifications

- **Type**: 3.7V 1200mAh Lithium-Polymer (LiPo) battery
- **Dimensions**: 50mm x 30mm x 5mm, selected to fit within the enclosure
- **Weight**: Approximately 25g, keeping the wand lightweight

### Power Management Considerations

1. **Runtime**: The selected battery provides approximately 7-8 hours of continuous use, sufficient for extended play sessions.

2. **Charging Solution**: The design includes a micro-USB charging port accessible through a small opening, allowing recharging without disassembly.

3. **Power Efficiency**: The code implemented on the ESP32 includes power-saving features such as:
   - Sleep mode when not actively recognizing gestures
   - Efficient sampling rates to minimize power consumption
   - LED indicator programmed to minimize power usage

4. **Battery Protection**: Includes over-discharge and over-charge protection circuits to extend battery life and ensure safety.

## Manufacturing Process

The enclosure was created through the following process:

1. **3D Design**: Initial concept created in 3D modeling software (as shown in the first image)
2. **Laser Cutting**: Chipboard pieces precision laser-cut based on the design templates, ensuring clean edges and accurate dimensions
3. **Assembly**: Components carefully assembled with super glue (cyanoacrylate), ensuring strong and durable bonds between all chipboard pieces
4. **Electronics Integration**: Circuit board, sensor, and battery mounted with appropriate spacing and securing mechanisms
5. **Testing**: Rigorous testing conducted to ensure durability and functionality
6. **Finishing**: Edges smoothed and structure reinforced where needed

## Functional Advantages

This enclosure design offers several advantages for the Magic Wand project:

1. **Gesture Performance**: The elongated handle facilitates clear, distinct gestures that are easily recognizable by the ML model.

2. **Consistent Sensor Orientation**: The fixed mounting position ensures consistent data collection.

3. **User Feedback**: The LED indicator provides immediate feedback on wand status and successful gesture recognition.

4. **Accessibility**: Simple construction allows for easy maintenance and component replacement if needed.

5. **Thematic Coherence**: The design complements the magical nature of the project, enhancing the user experience.

## Future Improvements

Potential enhancements for future iterations:

1. **Material Upgrade**: Consider upgrading to 3D printed PLA for increased durability and precision.

2. **Waterproofing**: Add water-resistant coating for protection against moisture.

3. **Additional Indicators**: Incorporate RGB LEDs to display different colors for each recognized spell.

4. **Haptic Feedback**: Add a small vibration motor to provide tactile feedback when gestures are recognized.

5. **Wireless Charging**: Implement Qi wireless charging capability to eliminate the need for a charging port.
