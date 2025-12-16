// Static Quiz Data for Physical AI & Humanoid Robotics Handbook
export const quizData = {
  'Introduction to Physical AI': [
    {
      question: "What is the main difference between Embodied Intelligence and Digital AI?",
      options: [
        "Embodied Intelligence runs on physical hardware, while Digital AI runs on virtual systems",
        "Embodied Intelligence learns through physical interaction with the real world, while Digital AI processes abstract data",
        "There is no significant difference between them",
        "Embodied Intelligence is faster than Digital AI"
      ],
      correctAnswer: 1,
      explanation: "Embodied Intelligence involves learning and decision-making through physical interaction with the real world, while Digital AI typically processes abstract data without physical embodiment."
    },
    {
      question: "Which of the following is a key challenge in Physical AI that doesn't exist in traditional AI?",
      options: [
        "Processing speed",
        "Data storage requirements",
        "Real-world physics and embodiment constraints",
        "Algorithm complexity"
      ],
      correctAnswer: 2,
      explanation: "Physical AI must deal with real-world physics, embodiment constraints, and the complexities of interacting with the physical environment, which traditional AI doesn't face."
    },
    {
      question: "Why is the concept of embodiment critical to Physical AI?",
      options: [
        "It makes the AI run faster",
        "It allows the AI to learn through physical interaction and sensorimotor experiences",
        "It reduces computational requirements",
        "It makes the AI more secure"
      ],
      correctAnswer: 1,
      explanation: "Embodiment is critical because it enables the AI to learn through physical interaction, sensorimotor experiences, and real-world feedback, which is fundamental to Physical AI."
    }
  ],
  'ROS 2 Fundamentals 1: Nodes, Topics, and Services': [
    {
      question: "What is a Node in ROS 2?",
      options: [
        "A physical robot component",
        "A process that performs computation and can subscribe to/publish topics",
        "A type of sensor",
        "A communication protocol"
      ],
      correctAnswer: 1,
      explanation: "In ROS 2, a Node is a process that performs computation. Nodes are the fundamental executable units in the ROS graph and can subscribe to or publish topics."
    },
    {
      question: "What is the purpose of Topics in ROS 2?",
      options: [
        "To store permanent data",
        "To provide a communication channel for messages between nodes",
        "To define the physical structure of a robot",
        "To manage hardware components"
      ],
      correctAnswer: 1,
      explanation: "Topics in ROS 2 provide a communication channel for messages between nodes. They enable asynchronous communication where publishers send messages and subscribers receive them."
    },
    {
      question: "What is the role of a Service in ROS 2?",
      options: [
        "To provide continuous data streams",
        "To enable request-response communication between nodes",
        "To store configuration data",
        "To manage network connections"
      ],
      correctAnswer: 1,
      explanation: "Services in ROS 2 provide a request-response communication pattern, where a client sends a request and a server responds with a result."
    }
  ],
  'ROS 2 Fundamentals 2: Python Agents & rclpy': [
    {
      question: "What is rclpy?",
      options: [
        "A Python visualization library",
        "The Python client library for ROS 2",
        "A Python testing framework",
        "A Python machine learning library"
      ],
      correctAnswer: 1,
      explanation: "rclpy is the Python client library for ROS 2 that provides Python bindings for the ROS 2 client library implementation."
    },
    {
      question: "Which of the following is a key advantage of using Python for ROS 2 development?",
      options: [
        "Faster execution speed",
        "Simpler syntax and rapid prototyping",
        "Better hardware control",
        "Lower memory usage"
      ],
      correctAnswer: 1,
      explanation: "Python offers simpler syntax and rapid prototyping capabilities, making it ideal for developing and testing ROS 2 nodes quickly."
    },
    {
      question: "What does a typical Python node in ROS 2 inherit from?",
      options: [
        "rclpy.Node",
        "rclpy.Robot",
        "rclpy.Agent",
        "rclpy.Publisher"
      ],
      correctAnswer: 0,
      explanation: "Python nodes in ROS 2 typically inherit from rclpy.Node, which provides the basic functionality for creating publishers, subscribers, services, etc."
    }
  ],
  'ROS 2 Fundamentals 3: URDF for Humanoids': [
    {
      question: "What does URDF stand for in robotics?",
      options: [
        "Unified Robot Description Format",
        "Universal Robot Development Framework",
        "Unified Robotics Design File",
        "Universal Robot Dynamics Format"
      ],
      correctAnswer: 0,
      explanation: "URDF stands for Unified Robot Description Format, which is an XML format used to describe robot models in ROS/Gazebo."
    },
    {
      question: "What is the primary purpose of URDF in humanoid robotics?",
      options: [
        "To control robot movements",
        "To describe the physical structure and kinematics of the robot",
        "To manage sensor data",
        "To handle communication between nodes"
      ],
      correctAnswer: 1,
      explanation: "URDF describes the physical structure, joint relationships, and kinematic properties of humanoid robots, enabling simulation and control."
    },
    {
      question: "Which element in URDF defines the physical properties of a robot link?",
      options: [
        "joint",
        "material",
        "link",
        "transmission"
      ],
      correctAnswer: 2,
      explanation: "The 'link' element in URDF defines a rigid body with physical properties like mass, inertia, and visual/collision geometry."
    }
  ],
  'Gazebo Simulation 1: Physics, Gravity, Collisions': [
    {
      question: "What is the primary physics engine used in Gazebo?",
      options: [
        "Bullet",
        "ODE (Open Dynamics Engine)",
        "PhysX",
        "Both Bullet and ODE"
      ],
      correctAnswer: 3,
      explanation: "Gazebo supports multiple physics engines including Bullet and ODE, allowing flexibility in simulation scenarios."
    },
    {
      question: "What role does gravity play in Gazebo simulations?",
      options: [
        "It only affects flying robots",
        "It provides realistic physics simulation for ground-based robots",
        "It is optional and not needed for most simulations",
        "It only affects sensors"
      ],
      correctAnswer: 1,
      explanation: "Gravity provides realistic physics simulation that affects robot dynamics, stability, and interaction with the environment."
    },
    {
      question: "What are collision properties in Gazebo used for?",
      options: [
        "To define visual appearance",
        "To detect contact between objects and simulate physical interactions",
        "To store robot configuration",
        "To manage communication"
      ],
      correctAnswer: 1,
      explanation: "Collision properties define how objects interact physically, enabling contact detection and realistic physical simulation."
    }
  ],
  'Gazebo Simulation 2: Unity Rendering & Human-Robot Interaction': [
    {
      question: "What is the primary benefit of realistic rendering in robotics simulation?",
      options: [
        "Makes the simulation look prettier",
        "Provides realistic sensor data for vision-based algorithms",
        "Increases simulation speed",
        "Reduces computational requirements"
      ],
      correctAnswer: 1,
      explanation: "Realistic rendering provides accurate sensor data for computer vision algorithms, enabling training and testing with realistic visual input."
    },
    {
      question: "What is human-robot interaction (HRI) in the context of simulation?",
      options: [
        "Programming robots to interact with humans",
        "Testing human-robot collaboration scenarios in a safe virtual environment",
        "Replacing humans with robots",
        "Making robots look like humans"
      ],
      correctAnswer: 1,
      explanation: "HRI in simulation allows for testing and developing human-robot collaboration scenarios in a safe, controlled virtual environment."
    },
    {
      question: "Which aspect of simulation is crucial for transferring vision-based algorithms to real robots?",
      options: [
        "Physics accuracy",
        "Rendering fidelity",
        "Communication protocols",
        "Node architecture"
      ],
      correctAnswer: 1,
      explanation: "Rendering fidelity is crucial for vision-based algorithms as it ensures the simulated camera data closely matches real-world sensor data."
    }
  ],
  'Isaac Sim 1: NVIDIA Isaac Sim & Synthetic Data': [
    {
      question: "What is the main advantage of synthetic data generation in Isaac Sim?",
      options: [
        "It is cheaper to produce",
        "It provides labeled training data for AI models without privacy concerns",
        "It requires less computational power",
        "It is easier to collect"
      ],
      correctAnswer: 1,
      explanation: "Synthetic data provides perfectly labeled training data without privacy concerns and with full control over scene parameters."
    },
    {
      question: "What makes NVIDIA Isaac Sim particularly powerful for robotics development?",
      options: [
        "Its simple interface",
        "Its integration with NVIDIA GPUs and AI frameworks",
        "Its low cost",
        "Its compatibility with older systems"
      ],
      correctAnswer: 1,
      explanation: "Isaac Sim leverages NVIDIA's GPU technology and AI frameworks for high-fidelity simulation and accelerated AI training."
    },
    {
      question: "What is synthetic data?",
      options: [
        "Data collected from real robots",
        "Artificially generated data that mimics real-world data",
        "Data from simulations that is not realistic",
        "Outdated sensor data"
      ],
      correctAnswer: 1,
      explanation: "Synthetic data is artificially generated data that mimics real-world data patterns, useful for training AI models safely."
    }
  ],
  'Isaac Sim 2: Isaac ROS & Hardware Accelerated VSLAM': [
    {
      question: "What does VSLAM stand for?",
      options: [
        "Visual Sensor Localization and Mapping",
        "Vision-Based Simultaneous Localization and Mapping",
        "Virtual Sensor Linear Algebra Methods",
        "Variable Speed Linear Actuation Motor"
      ],
      correctAnswer: 1,
      explanation: "VSLAM stands for Vision-Based Simultaneous Localization and Mapping, using visual sensors for navigation and mapping."
    },
    {
      question: "What is Isaac ROS?",
      options: [
        "A version of ROS specifically for Isaac Sim",
        "A collection of ROS packages that accelerate robotics applications using NVIDIA hardware",
        "A new version of ROS",
        "A competitor to ROS"
      ],
      correctAnswer: 1,
      explanation: "Isaac ROS is a collection of ROS packages that leverage NVIDIA hardware acceleration for robotics applications."
    },
    {
      question: "Why is hardware acceleration important for VSLAM?",
      options: [
        "It reduces the need for sensors",
        "It provides the computational power needed for real-time visual processing",
        "It simplifies the algorithms",
        "It eliminates the need for calibration"
      ],
      correctAnswer: 1,
      explanation: "VSLAM requires intensive real-time visual processing that benefits significantly from hardware acceleration for practical deployment."
    }
  ],
  'Isaac Sim 3: Nav2 for Bipedal Movement': [
    {
      question: "What is Nav2?",
      options: [
        "A new version of ROS",
        "The navigation stack for ROS 2",
        "A simulation environment",
        "A robot controller"
      ],
      correctAnswer: 1,
      explanation: "Nav2 is the navigation stack for ROS 2, providing path planning and navigation capabilities for robots."
    },
    {
      question: "What makes bipedal navigation more challenging than wheeled navigation?",
      options: [
        "Speed limitations",
        "Balance and stability requirements",
        "Sensor limitations",
        "Communication issues"
      ],
      correctAnswer: 1,
      explanation: "Bipedal robots must maintain balance and stability while navigating, making path planning and execution more complex."
    },
    {
      question: "What is a key consideration for Nav2 when planning paths for humanoid robots?",
      options: [
        "Battery life",
        "Footstep planning and balance constraints",
        "Communication range",
        "Sensor fusion"
      ],
      correctAnswer: 1,
      explanation: "Humanoid robots require footstep planning that considers balance and stability constraints, unlike wheeled robots."
    }
  ],
  'VLA Voice Action: Whisper & Voice Commands': [
    {
      question: "What is Whisper in the context of voice processing?",
      options: [
        "A quiet mode for robots",
        "An open-source speech recognition model by OpenAI",
        "A type of microphone",
        "A communication protocol"
      ],
      correctAnswer: 1,
      explanation: "Whisper is OpenAI's open-source automatic speech recognition (ASR) system that can transcribe and translate speech."
    },
    {
      question: "What does VLA stand for in robotics?",
      options: [
        "Voice-Linked Automation",
        "Vision-Language-Action",
        "Variable Linear Actuator",
        "Virtual Laboratory Assistant"
      ],
      correctAnswer: 1,
      explanation: "VLA stands for Vision-Language-Action, referring to systems that integrate visual perception, language understanding, and robotic action."
    },
    {
      question: "What is a key challenge in implementing voice commands for robots?",
      options: [
        "Cost of microphones",
        "Noise filtering and context understanding",
        "Limited vocabulary",
        "Communication protocols"
      ],
      correctAnswer: 1,
      explanation: "Robots must filter background noise and understand commands in context, which is computationally challenging."
    }
  ],
  'VLA Cognitive Planning: LLMs to ROS 2 Actions': [
    {
      question: "What does LLM stand for in robotics?",
      options: [
        "Low-Level Motion",
        "Large Language Model",
        "Linear Learning Machine",
        "Legged Locomotion Module"
      ],
      correctAnswer: 1,
      explanation: "LLM stands for Large Language Model, which can process natural language and generate structured outputs for robotics."
    },
    {
      question: "What is cognitive planning in robotics?",
      options: [
        "Planning using only sensors",
        "High-level planning that incorporates reasoning and understanding",
        "Simple path planning",
        "Motor control planning"
      ],
      correctAnswer: 1,
      explanation: "Cognitive planning involves high-level reasoning and understanding to break down complex tasks into executable robot actions."
    },
    {
      question: "How do LLMs bridge the gap between natural language and ROS 2 actions?",
      options: [
        "They replace ROS 2",
        "They translate high-level language commands into sequences of ROS 2 actions",
        "They improve communication speed",
        "They handle sensor fusion"
      ],
      correctAnswer: 1,
      explanation: "LLMs interpret natural language commands and generate appropriate sequences of ROS 2 actions to execute the requested task."
    }
  ],
  'Sim-to-Real Transfer': [
    {
      question: "What is the 'reality gap' in robotics?",
      options: [
        "The difference between simulation and reality",
        "The gap between different robots",
        "The time delay in communication",
        "The difference in programming languages"
      ],
      correctAnswer: 0,
      explanation: "The reality gap refers to the difference between simulated environments and the real world, often causing models trained in simulation to fail when deployed on actual hardware."
    },
    {
      question: "What is domain randomization?",
      options: [
        "Randomizing robot hardware",
        "Introducing variations in simulation parameters to improve real-world transfer",
        "Randomizing communication protocols",
        "Changing programming languages randomly"
      ],
      correctAnswer: 1,
      explanation: "Domain randomization involves varying simulation parameters widely to make models more robust to the differences between simulation and reality."
    },
    {
      question: "Why is sim-to-real transfer important?",
      options: [
        "It reduces the need for programming",
        "It allows safe and efficient development and testing before real-world deployment",
        "It makes robots faster",
        "It reduces hardware costs"
      ],
      correctAnswer: 1,
      explanation: "Sim-to-real transfer enables safe, cost-effective development and testing in simulation before deploying to real robots, reducing risks and costs."
    }
  ],
  'Hardware Lab Setup': [
    {
      question: "What is a critical factor when designing a humanoid robotics laboratory?",
      options: [
        "Only aesthetic considerations",
        "Safety measures and workspace layout for large, mobile robots",
        "Minimal lighting requirements",
        "Reduced ventilation needs"
      ],
      correctAnswer: 1,
      explanation: "Humanoid robots require special safety considerations and spacious layouts to accommodate their movement and potential fall scenarios."
    },
    {
      question: "What type of flooring is typically recommended for humanoid robotics labs?",
      options: [
        "Highly polished surfaces",
        "Non-slip surfaces with shock absorption",
        "Carpeted surfaces",
        "Metallic surfaces"
      ],
      correctAnswer: 1,
      explanation: "Non-slip surfaces with shock absorption protect both robots and humans during operation and potential falls."
    },
    {
      question: "Why is networking infrastructure critical in a robotics lab?",
      options: [
        "For internet browsing",
        "For robot-to-robot and robot-to-control communication",
        "For video streaming",
        "For power distribution"
      ],
      correctAnswer: 1,
      explanation: "Robots need reliable network infrastructure for communication, coordination, and data transfer during operation."
    }
  ],
  'Capstone: Autonomous Humanoid': [
    {
      question: "What is a key integration challenge in autonomous humanoid systems?",
      options: [
        "Choosing colors",
        "Combining perception, planning, control, and actuation into a cohesive system",
        "Minimizing weight",
        "Reducing cost"
      ],
      correctAnswer: 1,
      explanation: "Autonomous humanoid systems require tight integration of perception, planning, control, and actuation subsystems for coordinated operation."
    },
    {
      question: "What does autonomy mean in the context of humanoid robots?",
      options: [
        "Complete independence from humans",
        "The ability to operate without continuous human intervention while adapting to the environment",
        "Complete removal of safety systems",
        "Maximum speed operation"
      ],
      correctAnswer: 1,
      explanation: "Autonomy in humanoid robots means operating independently while adapting to environmental changes and task requirements."
    },
    {
      question: "What is a critical requirement for autonomous humanoid operation?",
      options: [
        "Maximum speed",
        "Robust perception and real-time decision-making capabilities",
        "Minimal sensors",
        "Simple programming"
      ],
      correctAnswer: 1,
      explanation: "Autonomous humanoid robots need robust perception systems and real-time decision-making to navigate and interact safely in dynamic environments."
    }
  ]
};