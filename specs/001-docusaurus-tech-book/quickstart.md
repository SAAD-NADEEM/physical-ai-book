# Quickstart Guide: Docusaurus-based Technical Book on Physical AI and Humanoid Robotics

## Overview
This guide provides a quick introduction to setting up, building, and contributing to the Docusaurus-based technical book on Physical AI and Humanoid Robotics. This book is designed for a 13-week graduate course covering ROS 2, Gazebo/Unity simulation, NVIDIA Isaac, and Vision-Language-Action integration.

## Prerequisites
- Node.js version 18.0 or higher
- npm or yarn package manager
- Git for version control
- GitHub account (for contributing)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-robotics-book.git
cd physical-ai-robotics-book/website
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Development Server
```bash
npm run start
```
This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without restarting the server.

## Project Structure
```
website/
├── blog/                # Blog posts related to Physical AI and Robotics
├── docs/                # Main documentation content organized by modules
│   ├── module-1-ros2/   # Module 1 content (Weeks 1-5)
│   │   ├── chapter-1/   # Individual chapters
│   │   ├── chapter-2/
│   │   ├── ...
│   │   └── chapter-5/
│   ├── module-2-simulation/ # Module 2 content (Weeks 6-7)
│   │   ├── chapter-6/
│   │   └── chapter-7/
│   ├── module-3-isaac/  # Module 3 content (Weeks 8-10)
│   │   ├── chapter-8/
│   │   ├── chapter-9/
│   │   └── chapter-10/
│   └── module-4-vla/    # Module 4 content (Weeks 11-13)
│       ├── chapter-11/
│       ├── chapter-12/
│       └── chapter-13/
├── src/
│   ├── components/      # Custom React components for book features
│   ├── css/             # Custom styles
│   └── pages/           # Additional pages
├── static/              # Static assets like images, diagrams
├── docusaurus.config.js # Main configuration file
├── sidebars.js          # Navigation structure
├── package.json         # Dependencies and scripts
└── README.md            # Project overview
```

## Adding Content

### Creating a New Chapter
1. Navigate to the appropriate module directory in `website/docs/`
2. Create a new directory for your chapter with a descriptive name
3. Add a `index.md` file with the content

### Chapter Template
```markdown
---
title: Chapter Title
description: Brief description of the chapter content
keywords: [list, of, relevant, keywords]
sidebar_position: 1
---

# Chapter Title

## Learning Objectives
After completing this chapter, students will be able to:
- Objective 1
- Objective 2
- Objective 3

## Prerequisites
Before starting this chapter, students should:
- Prerequisite 1
- Prerequisite 2

## Core Concepts
Content explaining the core concepts...

## Implementation
Step-by-step implementation guide...

## Code Examples
Code examples with explanations of what and why...

## Summary
Brief summary of key points covered in the chapter.

## Exercises
1. **Logical Exercise**: Question requiring logical reasoning.
2. **Conceptual Exercise**: Question testing conceptual understanding.
3. **Implementation Exercise**: Hands-on implementation task.
```

## Building the Site

### Local Build
```bash
npm run build
```
This command generates static content in the `build` directory and can be served using any static hosting service.

### Testing Build Locally
```bash
npm run serve
```
This command builds the site and serves the generated content locally for testing.

## Configuration

### Site Metadata
Update `docusaurus.config.js` to adjust:
- Site title and tagline
- GitHub Pages deployment settings
- Navigation items
- Footer links

### Navigation
Edit `sidebars.js` to update the sidebar navigation structure, organizing modules and chapters hierarchically.

## Contributing

### Content Guidelines
1. All content must comply with the constitutional principles
2. Every technical term must be defined in glossary.md
3. All code examples must be tested and functional
4. All mathematical equations must be validated
5. All external concepts must be cited in APA format
6. All dependencies must have explicit version numbers

### Documentation Standards
- Diagrams required for spatial concepts, system architectures, and multi-step processes
- Mathematical notation defined in docs/notation.md and used consistently
- All facts must be traceable to sources
- 0% tolerance on plagiarism
- All claims verified against sources

## Deployment
The site is configured for GitHub Pages deployment. After merging changes to the main branch, GitHub Actions will automatically build and deploy the site to the configured GitHub Pages URL.

### Manual Deployment
If needed, deploy manually using:
```bash
npm run deploy
```

## Quality Gates
Before submitting content, ensure:
- Docusaurus builds successfully with zero errors/warnings
- All internal and external links are valid
- Spell check passes with strict verification
- All code examples run successfully in test environments
- All constitutional principles are followed