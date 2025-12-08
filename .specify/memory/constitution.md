<!--
Sync Impact Report:

- Version change: 1.0.0 -> 2.0.0
- List of modified principles: All principles were updated with project-specific details.
- Added sections: None (placeholders filled).
- Removed sections: None.
- Templates requiring updates:
    - .specify/templates/plan-template.md: ✅ updated (no direct changes, structure is compatible)
    - .specify/templates/spec-template.md: ✅ updated (no direct changes, structure is compatible)
    - .specify/templates/tasks-template.md: ✅ updated (modified to enforce mandatory testing)
    - .claude/commands/sp.adr.md: ✅ updated
    - .claude/commands/sp.analyze.md: ✅ updated
    - .claude/commands/sp.checklist.md: ✅ updated
    - .claude/commands/sp.constitution.md: ✅ updated
    - .claude/commands/sp.git.commit_pr.md: ✅ updated
    - .claude/commands/sp.implement.md: ✅ updated
    - .claude/commands/sp.phr.md: ✅ updated
    - .claude/commands/sp.plan.md: ✅ updated
    - .claude/commands/sp.specify.md: ✅ updated
    - .claude/commands/sp.tasks.md: ✅ updated (modified to enforce mandatory testing)
- Follow-up TODOs: None.
-->
# Physical AI & Humanoid Robotics — Docusaurus Book + Integrated RAG Chatbot Constitution

## Core Principles

### I. Technical Accuracy and Reproducibility
- All technical content must be accurate, referencing official documentation or verified sources (ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA, RAG).
- All code examples must be validated and run as written within the specified robotics toolchain (ROS 2, rclpy, URDF, simulation pipelines).
- All steps must be reproducible with correct tooling and setup instructions provided.

### II. Instructional Clarity
Writing must be clear, concise, and targeted at learners in the robotics and AI fields.

### III. Grounded and Non-Hallucinated Content
- The integrated RAG chatbot must provide answers grounded strictly in the book's content or user-selected text.
- Hallucination beyond the provided knowledge base is strictly prohibited for the chatbot.

### IV. Modular and Buildable Structure
- The Docusaurus site structure must be modular to accommodate Modules 1–4 and the Capstone project.
- The entire Docusaurus book must build and deploy cleanly to GitHub Pages.

## Key Standards and Constraints

- **Technology Stack**: Python and TypeScript are the only approved languages. The stack includes OpenAI Agents/ChatKit, FastAPI, and Qdrant Cloud (Free Tier).
- **Content Sourcing**: All information must be referenced from official documentation or other verified sources.
- **RAG Chatbot Functionality**: The chatbot must support two modes: querying the full book content and querying only user-selected text.
- **Project Scope**: The book must include Modules 1–4 and a Capstone project demonstrating a Voice → Plan → Navigate → Identify → Manipulate workflow.

## Success Criteria

- The Docusaurus site builds and deploys without errors.
- The RAG chatbot provides accurate, grounded answers with source attribution.
- All robotics simulations and code examples are fully reproducible by a user following the instructions.
- The capstone project workflow is fully documented and functional as described.

## Governance

- All contributions must adhere to the principles of technical accuracy, clarity, and reproducibility.
- Changes to the core technology stack (ROS 2, Python, TypeScript, Docusaurus, OpenAI Agents/ChatKit, FastAPI, Qdrant) require a formal ADR.

**Version**: 2.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08