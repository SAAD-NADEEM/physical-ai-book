# Data Model: Docusaurus-based Technical Book on Physical AI and Humanoid Robotics

## Overview

This document defines the data model for the technical book, focusing on the content structure, metadata schema, and organization of information within the Docusaurus-based site.

## Content Structure

### Book Level Entities

#### Book
- **Entity**: Book
- **Fields**:
  - title: string (Physical AI and Humanoid Robotics)
  - subtitle: string (A 13-Week Graduate Course)
  - description: string (Comprehensive technical book covering ROS 2, Simulation, NVIDIA Isaac, and VLA)
  - author: string[]
  - publisher: string
  - year: integer
  - edition: string
  - totalWords: range (40000-60000)
  - modulesCount: integer (4)
  - chaptersCount: integer (13+)
- **Relationships**: Contains Modules, References Glossary, References Notation

### Module Level Entities

#### Module
- **Entity**: Module
- **Fields**:
  - moduleId: string (unique identifier like "module-1-ros2")
  - title: string (descriptive module title)
  - description: string (overview of module content)
  - duration: string (week range, e.g., "Weeks 1-5")
  - prerequisites: string[] (prerequisite modules or knowledge areas)
  - learningObjectives: string[] (measurable learning outcomes)
  - chapters: integer (number of chapters in the module)
  - estimatedReadingTime: integer (hours)
- **Relationships**: Contains Chapters, Belongs to Book

### Chapter Level Entities

#### Chapter
- **Entity**: Chapter
- **Fields**:
  - chapterId: string (unique identifier like "chapter-1")
  - moduleId: string (reference to parent module)
  - title: string (chapter title)
  - description: string (brief chapter summary)
  - position: integer (order within module)
  - prerequisites: string[] (specific chapter or concept prerequisites)
  - learningObjectives: string[] (measurable learning outcomes)
  - content: string (the chapter content in Markdown)
  - keywords: string[] (SEO keywords)
  - estimatedReadingTime: integer (minutes)
  - exercises: Exercise[]
- **Relationships**: Belongs to Module, References Glossary terms, References Notation

#### Exercise
- **Entity**: Exercise
- **Fields**:
  - exerciseId: string (unique identifier)
  - chapterId: string (reference to parent chapter)
  - title: string
  - type: enum (Logical, Conceptual, Implementation)
  - difficulty: enum (Basic, Intermediate, Advanced)
  - description: string
  - solution: string (optional sample solution)
  - successCriteria: string
- **Relationships**: Belongs to Chapter

### Supporting Document Entities

#### GlossaryTerm
- **Entity**: GlossaryTerm
- **Fields**:
  - termId: string (unique identifier)
  - term: string (the term being defined)
  - definition: string (clear, concise definition)
  - category: string (classification like "Software", "Hardware", "Concept", "Algorithm")
  - seeAlso: string[] (related terms)
  - firstAppearance: string (first chapter where term appears)
- **Relationships**: Referenced by Chapters

#### NotationSymbol
- **Entity**: NotationSymbol
- **Fields**:
  - symbolId: string (unique identifier like "math-001")
  - symbol: string (the actual mathematical notation)
  - description: string (what the symbol represents)
  - context: string (where it's typically used)
  - example: string (usage example)
  - firstAppearance: string (first chapter where notation appears)
- **Relationships**: Referenced by Chapters

## Metadata Schema

### Frontmatter Schema for All Pages

All Markdown files in the documentation must include the following frontmatter metadata:

```yaml
title: String # Page title
description: String # Brief description for SEO and social sharing
keywords: Array<String> # SEO-relevant keywords
sidebar_position: Integer # Position in sidebar navigation
tags: Array<String> # Additional classification tags
authors: Array # Authors of the content
```

### Specialized Metadata

#### Module Page Frontmatter
```yaml
title: String
description: String
keywords: Array<String>
sidebar_position: Integer
duration: String # e.g., "5 weeks"
prerequisites: Array<String> # Links to prerequisite content
learning_outcomes: Array<String>
table_of_contents: Boolean
```

#### Chapter Page Frontmatter
```yaml
title: String
description: String
keywords: Array<String>
sidebar_position: Integer
module_ref: String # Reference to parent module
prerequisites: Array<String> # Specific prerequisites for this chapter
learning_objectives: Array<String>
estimated_reading_time: Integer # In minutes
exercises_count: Integer
```

## Content Relationships

### Navigation Hierarchy
```
Book
├── Module 1 (ROS 2 Fundamentals)
│   ├── Chapter 1 (Introduction to ROS 2)
│   ├── Chapter 2 (Nodes, Topics, Services, Actions)
│   ├── Chapter 3 (Robot Manipulation and Control)
│   ├── Chapter 4 (Navigation and Path Planning)
│   └── Chapter 5 (Multi-Robot Systems)
├── Module 2 (Simulation Environments)
│   ├── Chapter 6 (Physics Simulation with Gazebo)
│   └── Chapter 7 (Unity Integration)
├── Module 3 (NVIDIA Isaac Platform)
│   ├── Chapter 8 (Isaac Sim and Isaac ROS)
│   ├── Chapter 9 (Perception and Sensing)
│   └── Chapter 10 (Navigation and Manipulation)
└── Module 4 (Vision-Language-Action Integration)
    ├── Chapter 11 (Vision-Language Models)
    ├── Chapter 12 (Action Generation and Execution)
    └── Chapter 13 (Capstone Project)
```

### Cross-Reference Patterns
- Chapters reference Glossary terms using standardized IDs
- Chapters reference Notation symbols using standardized IDs
- Modules reference other modules for cross-module concepts
- Chapters link to exercises within the same chapter
- Exercises may reference concepts from multiple chapters

## Validation Rules from Requirements

### Required Content Elements
1. Each Chapter must include:
   - Learning Objectives (measurable)
   - Prerequisites (explicit)
   - Core Concepts section
   - Implementation section with code examples
   - Functional code examples (tested, commented)
   - Summary section
   - 3 exercises (logical, conceptual, implementation)
   - Diagrams for spatial/architectural concepts
   - Version specifications for all dependencies
   - APA citations where needed

### Quality Constraints
1. All code examples must be tested and functional
2. All technical terms must be defined in the glossary
3. All mathematical notation must be defined in the notation document
4. All external dependencies must have version numbers specified
5. All citations must follow APA format
6. All pages must include complete frontmatter metadata

### Structural Constraints
1. Content must follow the 4-module, 13-chapter structure
2. Modules must map to the 13-week course timeline
3. Each module must have a clear introduction page
4. Navigation must be hierarchical (Module → Chapter → Section)
5. All content must be accessible via sidebar navigation

## State Transitions

### Chapter State Model
- Draft: Created but not reviewed
- In Review: Undergoing technical review
- Approved: Reviewed and approved for publication
- Published: Included in the live site
- Deprecated: Content is outdated but kept for historical reference

## Access Patterns

### Common Queries
1. Find all chapters in a module
2. Find all glossary terms referenced in a chapter
3. Find prerequisites for a specific chapter
4. Find exercises of a specific type (logical, conceptual, implementation)
5. Find all chapters referencing a specific notation symbol

### Indexing Recommendations
- Index by module and chapter for navigation efficiency
- Index by glossary term for cross-referencing
- Index by keyword for search functionality
- Index by author for attribution