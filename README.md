# 🤖 Physical AI & Humanoid Robotics Handbook (Agentic AI Edition)

> **The World's First Interactive & Agentic AI-Powered Textbook for Robotics.**
> *Bridging the gap between static content and active learning.*

![Project Banner](https://via.placeholder.com/1200x400.png?text=Physical+AI+Handbook+Banner)

---

## 🚀 Live Demo
**Click here to use the app:**
👉 **[View Live Site](https://anam258.github.io/physical-ai-book/)** *(Note: Since the backend runs on free tiers (Render), please allow 30-50 seconds for the first request to wake up the server!)*

---

## 🌟 Introduction

Welcome to the **Physical AI Handbook**, a next-generation educational platform built for the **Governor Sindh IT Initiative (Agentic AI Course)**. 

Unlike traditional textbooks, this platform doesn't just display text—it **understands** the reader. Engineered using the advanced reasoning capabilities of **Claude** and the **Speckit-Plus** framework, this Agentic AI solution adapts content dynamically, translates technical concepts into native Urdu without losing accuracy, and offers real-time mentorship via an embedded chatbot.

## ✨ Key Features (The "Wow" Factor)

### 1. 🇵🇰 Smart Technical Urdu Translation
We solved the language barrier without sacrificing technical accuracy.
- **Problem:** Google Translate ruins code and technical terms (e.g., changing "ROS 2 Nodes" to "Guthis").
- **Our Solution:** An intelligent agent that translates concepts into **Formal Urdu** while keeping keywords like `ROS 2`, `Python`, `Lidar`, and `C++` in English.

### 2. 🤖 AI Assistant (Context-Aware)
Stuck on a concept? Just ask!
- **Integrated Workflow:** Select any text on the page and click **"Ask AI"**.
- **Smart Context:** The chatbot instantly reads the selected text and explains it simply.
- **Sticky UI:** Always accessible via a glassmorphism floating widget.

### 3. 🧠 Adaptive Personalization
The book changes based on **WHO** is reading.
- **Dynamic Content:** At the start of every chapter, the AI analyzes the topic and the user's background.
- **Example:** A "Python Developer" gets code-heavy analogies, while a "Beginner" gets simple real-world examples.

### 4. 📝 Infinite Interactive Quizzes
Learning checks that are actually fun.
- **Instant Feedback:** Green/Red indicators with confetti 🎉 for correct answers.
- **Smart Retry:** Allows users to learn from mistakes without reloading.
- **Zero-Latency:** Optimized for instant loading across all chapters.

### 5. 🎨 Modern Dark Glassmorphism UI
- **Aesthetic:** Premium Dark Navy background with Neon Cyan accents.
- **Experience:** Smooth "Reading Progress Bar" on top, translucent footers, and responsive layouts.

---

## 🛠️ Tech Stack

- **Frontend:** React, Docusaurus v3 (Custom Swizzled Components)
- **Backend:** Python (FastAPI/Flask), OpenAI API
- **Styling:** CSS Modules (Glassmorphism & Neumorphism)
- **AI Agents:** Custom prompts for Translation, Personalization, and Quiz Logic.

---

## 🚀 How to Run Locally

### Prerequisites
- Node.js & npm
- Python 3.8+
- OpenAI API Key

### 1. Clone the Repository
```bash
git clone [https://github.com/Anam258/physical-ai-book.git](https://github.com/Anam258/physical-ai-book.git)
cd physical-ai-book

### 2. Setup Backend (The Brain) 

cd backend
pip install -r requirements.txt
# Create a .env file and add your OPENAI_API_KEY
python server.py

### 3. Setup Frontend (The Face)

cd frontend
npm install
npm start


Built with ❤️ by Anum Anwer for the Agentic AI Hackathon.

Copyright © 2025 Physical AI Handbook. All rights reserved.

