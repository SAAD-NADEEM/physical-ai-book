# Data Model: Docusaurus-based Technical Book on Physical AI and Humanoid Robotics

## Entity: Module
- **Description**: A major section of the book that spans multiple weeks of curriculum
- **Attributes**:
  - id: Unique identifier (e.g., "module-1-ros2")
  - title: Display title (e.g., "The Robotic Nervous System (ROS 2)")
  - weeks: Number of weeks covered (e.g., 5 for Module 1)
  - chapters: List of chapter identifiers within the module
  - introduction: Brief description of the module's content and objectives
  - prerequisites: List of prerequisite modules or knowledge areas
- **Relationships**:
  - Contains multiple Chapter entities
  - May have dependencies on other Module entities
- **Validation rules**:
  - Must have a title between 5 and 100 characters
  - Must specify a valid number of weeks
  - Must include at least one chapter

## Entity: Chapter
- **Description**: A weekly section within a module that contains specific learning objectives, prerequisites, content, and exercises
- **Attributes**:
  - id: Unique identifier (e.g., "module-1-chapter-1")
  - title: Display title
  - module_id: Reference to the parent Module
  - week_number: The week in the course this chapter covers
  - learning_objectives: List of measurable learning objectives
  - prerequisites: List of prerequisite chapters or concepts
  - content: Main content body (Markdown format)
  - summary: Brief summary of the chapter content
  - exercises: List of Exercise entities
  - sidebar_position: Navigation position in the sidebar
  - description: Meta description for SEO
  - keywords: List of keywords for search optimization
- **Relationships**:
  - Belongs to one Module entity
  - Contains multiple Exercise entities
  - May reference multiple Code Example entities
- **Validation rules**:
  - Must have learning objectives (minimum 1, maximum 5)
  - Must include prerequisites
  - Must include at least 3 exercises
  - Content must be between 1,500 and 3,000 words
  - Must include required frontmatter metadata

## Entity: Exercise
- **Description**: A learning activity within a chapter with logical, conceptual, or implementation scenarios
- **Attributes**:
  - id: Unique identifier (e.g., "chapter-1-exercise-1")
  - chapter_id: Reference to the parent Chapter
  - type: Category of exercise ("logical", "conceptual", or "implementation")
  - title: Brief title describing the exercise
  - description: Detailed description of the exercise requirements
  - success_criteria: What constitutes successful completion
  - sample_solution: Example solution (for educator reference)
- **Relationships**:
  - Belongs to one Chapter entity
- **Validation rules**:
  - Must have a defined type
  - Must include success criteria
  - Description must be between 50 and 500 words

## Entity: Code Example
- **Description**: A functional code snippet with comments explaining what it does and why it's implemented that way
- **Attributes**:
  - id: Unique identifier (e.g., "example-ros2-publisher")
  - chapter_id: Reference to the parent Chapter (optional, for examples not tied to specific chapters)
  - title: Brief descriptive title
  - code: The actual code content
  - language: Programming language or format (e.g., "python", "javascript", "bash", "urdf")
  - description: Explanation of what the code does
  - comments_explaining_what: Comments explaining what the code does
  - comments_explaining_why: Comments explaining why it's implemented this way
  - dependencies: List of required dependencies with version specifications
  - environment: Required runtime environment for testing
  - test_status: Whether the code example has been tested and verified (boolean)
- **Relationships**:
  - Belongs to one Chapter entity (optional)
- **Validation rules**:
  - Must include comments explaining both WHAT and WHY
  - Must specify dependencies with version numbers
  - Must be tested and verified before inclusion

## Entity: Glossary Term
- **Description**: A standardized term defined in glossary.md that maintains consistent terminology across the book
- **Attributes**:
  - id: Unique identifier (e.g., "ros2", "gazebo", "isaac")
  - term: The actual term being defined
  - definition: Clear and concise definition of the term
  - examples: Optional usage examples
  - see_also: List of related terms
  - category: The category this term belongs to (e.g., "software", "hardware", "algorithm")
- **Relationships**:
  - Referenced by multiple Chapter entities
- **Validation rules**:
  - Term and definition must not be empty
  - Must be linked from at least one content page

## Entity: Mathematical Notation
- **Description**: A symbol or formula defined in docs/notation.md and used consistently across the book
- **Attributes**:
  - id: Unique identifier (e.g., "rotation-matrix", "quaternion")
  - symbol: The actual mathematical notation (LaTeX format)
  - meaning: Clear explanation of what the notation represents
  - usage_context: Where this notation is typically used
  - examples: Examples of how the notation is applied
  - related_notations: List of related mathematical notations
- **Relationships**:
  - Referenced by multiple Chapter entities
- **Validation rules**:
  - Symbol and meaning must not be empty
  - Must include proper LaTeX formatting

## State Transitions

### Module State Transitions
- Draft → In Review → Approved → Published
- Can revert from any state back to Draft during editing process

### Chapter State Transitions
- Outline → Writing → In Review → Revising → Approved → Published
- Can move between states based on review feedback

### Code Example State Transitions
- Proposed → Implemented → Tested → Verified → Validated → Included in Content
- Failed testing returns to Implemented state for correction

## Additional Validation Rules
- All entities must have unique IDs within their type
- All Markdown files must include required frontmatter metadata (title, description, keywords, sidebar_position)
- All content must comply with constitutional principles (glossary enforcement, mathematical validation, etc.)
- All external links must be validated as part of the deployment process
- All code examples must be tested in appropriate environments before publication