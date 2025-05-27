🤖 Omni Directional Robot Platform - EN2160
Next-Generation Autonomous Mobile Robotics
<div align="center">
Show Image
Show Image
Show Image
Show Image
🏆 Engineering Excellence in Motion - Crafted by the Next Generation of Robotics Engineers
</div>

🌟 Project Vision
Revolutionizing Indoor Automation Through Intelligent Omnidirectional Mobility
This cutting-edge repository showcases the design and development of a state-of-the-art Omni Directional Robot Platform engineered as part of the prestigious EN2160 Electronic Design Realization Module at the University of Moratuwa. Our platform represents the perfect fusion of academic rigor and industry-grade performance, delivering unprecedented maneuverability in confined spaces.
<div align="center">
<!-- Add your hero image here -->
[Hero Image Placeholder - Main Robot Assembly]
</div>
✨ Core Philosophy

"Where precision meets innovation, and theory transforms into reality"

Our robot transcends traditional mobility constraints, offering 360-degree freedom of movement through advanced Mecanum wheel technology, intelligent control algorithms, and robust engineering practices.

🎯 Project Objectives
<div align="center">
🎪 Omnidirectional Mastery🧠 Intelligent Positioning⚙️ Precision Control4-wheel Mecanum platform for unrestricted motionAdvanced position estimation algorithmsReal-time wheel control with sub-millimeter accuracy
</div>
🚀 Key Achievements

360° Movement Freedom - Forward, backward, lateral, and rotational motion simultaneously
Industry-Grade Precision - Sub-millimeter positioning accuracy
Modular Architecture - Scalable design for diverse applications
Real-Time Performance - Ultra-low latency control systems


👥 Meet the Innovation Team
<div align="center">
The brilliant minds behind the next generation of autonomous robotics
</div>
🧑‍💻 Engineer📧 Contact🏷️ Index🎯 SpecializationPankaja Balasooriyabalasooriyabapi.22@uom.lk220054NSystems IntegrationOshani Dewasumithradewasumithrampo.22@uom.lk220112RControl AlgorithmsChandupa Dinesharadinesharamc.22@uom.lk220128VHardware DesignChamath Diunugaladiunugalach.22@uom.lk220143LEmbedded SystemsSahas Eshaneashansgs.22@uom.lk220148GSoftware ArchitectureRusiru Fernandofernandoard.22@uom.lk220161NPower ElectronicsBanuka Liyanageliyanagedlbb.22@uom.lk220362GMechanical DesignRusula Oshadha Pathiranapathiranapdro.22@uom.lk220448CTesting & Validation

🛠️ Revolutionary Features
<div align="center">
Where Innovation Meets Implementation
</div>
🎯 Core Capabilities
FeatureDescriptionStatus✅ Mecanum Wheel MasteryAdvanced 4-wheel omnidirectional locomotion system🟢 Complete✅ Multi-MCU ArchitectureDistributed processing with dedicated motor controllers🟢 Complete✅ Precision Sensing SuiteIMU-based orientation tracking and sensor fusion🟢 Complete✅ Wireless Command CenterBluetooth connectivity with intuitive GUI control🟢 Complete✅ Robust CommunicationRS485 differential signaling for noise immunity🟢 Complete✅ Modular PCB DesignCustom-engineered boards for power, logic, and motor control🟢 Complete✅ Real-Time DiagnosticsLive telemetry and system health monitoring🟢 Complete✅ Industrial AestheticsProfessional-grade chassis with automotive paint finish🟢 Complete
<div align="center">
<!-- Add feature showcase images here -->
[Feature Gallery - Control Interface, PCB Modules, Chassis Assembly]
</div>

🔧 Technical Excellence
🏗️ System Architecture
<div align="center">
<!-- Add system architecture diagram here -->
[Architecture Diagram Placeholder]
</div>
⚡ Hardware Specifications
ComponentSelectionPerformanceRationale🧠 Main MCUSAM3X8E (Arduino Due)84 MHz Cortex-M3High I/O density, robust processing power🎮 Motor ControllerSTM32F446RE180 MHz, Real-time loopsPrecision motor control with encoder feedback🔋 Drive Motors24V 5Nm Planetary Gear1:100 ratio, 120 RPMOptimal torque-speed characteristics🧭 Navigation IMUBosch BNO0559-DOF sensor fusionIndustry-standard orientation tracking⚡ Motor DriversVNH5019 H-Bridge30A peak currentRobust protection and thermal management🎯 Mecanum WheelsMetal-core constructionOmnidirectional mobilityEnhanced durability and load capacity🔌 Power System12V battery + regulationModular distributionFuse-protected multi-rail design📡 CommunicationRS485 + Bluetooth HC-05Differential + wirelessReliable inter-board and GUI connectivity
📐 Mechanical Design
<div align="center">
<!-- Add mechanical design images here -->
[Mechanical Assembly Views - Isometric, Side, Bottom]
</div>
Chassis Engineering:

Material: Stainless steel construction with cantalie sheet vibration dampening
Finish: Automotive-grade paint system for professional appearance
Assembly: In-house fabrication using precision welding and machining
Modularity: 3D printed enclosures for PCB protection and accessibility


🧠 Intelligence Stack
🎛️ Control Hierarchy
mermaidgraph TD
    A[Python GUI Controller] -->|Bluetooth| B[SAM3X8E Main Controller]
    B -->|RS485| C[STM32F446RE Motor Controller]
    B -->|I2C| D[BNO055 IMU]
    C -->|PWM| E[VNH5019 Motor Drivers]
    E --> F[Mecanum Wheels]
    D -->|Sensor Data| B
    C -->|Encoder Feedback| B
🎯 Software Architecture
LayerFunctionTechnology🖥️ User InterfaceIntuitive direction control and telemetryPython Tkinter GUI📡 CommunicationWireless command transmission and echo verificationBluetooth Serial Protocol🧠 High-Level ControlCommand parsing, sensor fusion, system coordinationSAM3X8E Embedded C++⚡ Real-Time ControlPWM generation, encoder processing, motor loopsSTM32 Real-Time OS🔧 Hardware AbstractionDriver interfaces and peripheral managementHAL Libraries

📊 Performance Metrics
<div align="center">
🏆 Benchmark Results
</div>
MetricSpecificationAchievement🏋️ Payload Capacity5 kg operational load✅ Exceeded⚖️ Total System Weight~25 kg including payload✅ Target Met🔧 Wheel Torque3.2 Nm per wheel minimum✅ Optimized🚀 Maximum Speed1 m/s target velocity✅ Achieved🎯 Positioning AccuracySub-millimeter precision✅ Validated📡 Communication Range10m+ Bluetooth connectivity✅ Confirmed⚡ Response Time<50ms command to motion✅ Real-Time

🔌 Electronic Systems
📋 PCB Module Ecosystem
<div align="center">
<!-- Add PCB images here -->
[PCB Gallery - Power Module, Main Controller, Motor Driver]
</div>
PCB ModuleFunctionKey Features🔋 Power DistributionMulti-rail power managementFuse protection, LED diagnostics, screw terminals🧠 Main ControllerSystem coordination and communication4-layer design, SAM3X8E, BNO055, HC-05 integration⚡ Motor ControllerReal-time motor drive and feedbackSTM32F446RE, VNH5019 drivers, RS485 communication
🛡️ Safety & Protection Systems

20A Master Fuse with branch-specific protection (2A-15A)
Thermal Management with active cooling consideration
Overcurrent Protection at component and system levels
EMI Mitigation through differential signaling and proper grounding


🚀 Applications & Use Cases
<div align="center">
Transforming Industries Through Intelligent Mobility
</div>
🏭 Industrial🏥 Healthcare🛒 Retail🎓 EducationLogistics automationHospital transportInventory managementResearch platformMaterial handlingSupply deliveryShelf restockingSTEM demonstrationsQuality inspectionPatient assistanceCustomer guidanceCompetition robotics

📁 Repository Architecture
OmniDirectionalRobot-EN2160/
├── 📁 firmware/              # Embedded systems code
│   ├── main_controller/      # SAM3X8E firmware
│   ├── motor_controller/     # STM32F446RE code  
│   └── libraries/           # Custom libraries and drivers
├── 📁 hardware/              # Electronic design files
│   ├── pcb_designs/         # KiCad/Altium schematics and layouts
│   ├── mechanical/          # CAD files and drawings
│   └── datasheets/          # Component specifications
├── 📁 software/              # High-level software
│   ├── gui_controller/      # Python Tkinter interface
│   ├── simulation/          # Motion simulation tools
│   └── utilities/           # Testing and calibration scripts
├── 📁 docs/                  # Comprehensive documentation
│   ├── design_reports/      # Technical documentation
│   ├── user_manual/         # Operation guidelines
│   └── api_reference/       # Software interfaces
├── 📁 media/                 # Visual assets
│   ├── images/              # Photos and diagrams
│   ├── videos/              # Demonstration footage
│   └── presentations/       # Project presentations
├── 📁 tests/                 # Validation and testing
│   ├── unit_tests/          # Component-level tests
│   ├── integration_tests/   # System-level validation
│   └── performance_data/    # Benchmark results
└── 📁 resources/            # Additional materials
    ├── references/          # Academic papers and standards
    └── tools/               # Development utilities

🌐 Future Roadmap
🔮 Next-Generation Enhancements
PhaseEnhancementImpactTimeline🔋 Phase 1Advanced Battery Management SystemExtended operational timeQ2 2025🤖 Phase 2Autonomous Navigation & SLAMSelf-directed operationQ3 2025🌐 Phase 3ROS2 Integration & Fleet ManagementMulti-robot coordinationQ4 2025🧠 Phase 4AI-Powered Decision MakingIntelligent task executionQ1 2026📡 Phase 5Cloud Analytics DashboardRemote monitoring and optimizationQ2 2026

🏆 Project Impact
<div align="center">
Engineering Excellence Recognition
</div>
This project demonstrates how academic excellence can drive industry innovation. By combining theoretical knowledge with practical implementation, we've created a platform that bridges the gap between education and real-world applications.
Key Contributions:

Advanced undergraduate-level robotics engineering
Open-source platform for educational advancement
Industry-applicable design methodologies
Sustainable and cost-effective innovation


📞 Connect With Us
<div align="center">
Join the Robotics Revolution
Show Image
Show Image
Show Image
Show Image
</div>

📄 Documentation & Resources
DocumentDescriptionAccess📋 Technical SpecificationsDetailed system parameters and performance data📖 View🔧 Assembly GuideStep-by-step construction instructions🛠️ Guide💻 Software ManualProgramming interfaces and API documentation💾 Manual🎥 Video DemonstrationsLive operation footage and feature showcases🎬 Watch📊 Performance AnalysisTesting results and benchmark comparisons📈 Data

🙏 Acknowledgments
<div align="center">
Standing on the Shoulders of Giants
</div>
We extend our deepest gratitude to:

University of Moratuwa - For providing world-class engineering education and facilities
Department of Electronic and Telecommunication Engineering - For academic guidance and resources
EN2160 Course Instructors - For mentorship and technical expertise
Industry Partners - For component support and technical consultation
Open Source Community - For foundational libraries and inspiration


📜 License & Usage
This project is released under the MIT License, promoting open innovation and educational advancement. We encourage the global robotics community to build upon our work, contribute improvements, and push the boundaries of autonomous mobile robotics.
MIT License - Open Innovation for Global Impact
Copyright (c) 2024 University of Moratuwa Robotics Team

<div align="center">
🌟 "Innovation Distinguished by Excellence" 🌟
Crafted with passion by the future engineers of tomorrow
University of Moratuwa | Department of Electronic and Telecommunication Engineering

Made with ❤️ in Sri Lanka | Engineered for the World
</div>
