<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (New constitution)
Added sections: All principles and sections for Physical AI and Humanoid Robotics
Removed sections: None
Templates requiring updates: ✅ updated (plan-template.md, spec-template.md, tasks-template.md)
Follow-up TODOs: None
-->

# Physical AI and Humanoid Robotics Constitution

## Core Principles

### I. Glossary Enforcement
Every term and concept must be defined in glossary.md to maintain consistent terminology across all documentation and implementations; All references to technical terms must link back to the glossary definition.

### II. Mathematical Validation
All mathematical equations, formulas, and theoretical principles must be validated through peer review or authoritative sources before implementation; Derivations must include step-by-step justifications showing correctness.

### III. Functional Code Examples
Every code example must be tested and proven functional in the intended environment; Code marked as pseudocode must be explicitly labeled as such with surrounding context.

### IV. Academic Citation Standard
Citations are required for all research findings, algorithms, and external concepts using APA format; Sources must be accessible and verifiable; Self-citation is prohibited without clear justification.

### V. Evidence-Based Technical Claims
Technical claims must be supported by either: (a) authoritative citations, (b) mathematical derivation/proof, or (c) reproducible experimental validation; Unsupported claims are prohibited.

### VI. Version Specification Compliance
All software dependencies, APIs, and frameworks must include explicit version numbers in documentation and implementation; Semantic versioning is required with clear deprecation policies.

## Standards

### Documentation Standards
- Diagrams REQUIRED for spatial concepts, system architectures, and multi-step processes
- Notation: Mathematical symbols defined in docs/notation.md and used consistently
- All facts must be traceable to sources
- Citation should be APA style
- 0% tolerance on Plagiarism
- All claims verified against sources

### Educational Structure Requirements
- Content MUST progress logically from fundamentals to advanced topics with clear learning pathways
- Each chapter/section MUST declare explicit prerequisites (prior chapters or external knowledge)
- Learning objectives MUST be measurable and stated at chapter start
- Each module follows prescribed pattern: introduction and chapter list
- Each chapter follows prescribed pattern: objectives, prerequisites, content (core concepts → implementation → examples), summary, exercises

### Exercise Framework
Each chapter must include at least 3 exercises covering: logical reasoning, conceptual understanding, and implementation scenarios; Exercises must have clear success criteria and sample solutions.

## Docusaurus Standards

### Navigation and Accessibility
- Whole book MUST be navigable and searchable via Docusaurus
- Metadata REQUIRED: title, description, keywords, sidebar_position in every .md frontmatter
- Search optimization: Keywords in headings, metadata, and first paragraph

### Performance and Deployment
- Book must build successfully before publication
- Static site deployment on GitHub Pages requires all resources to be local
- SEO compliance: automatic sitemap generation and proper robots.txt configuration

## Code Quality Requirements

### Example Code Standards
- Example code must be complete for the situation (fulfill required logic even if partial implementation)
- Code must include comments explaining both WHAT the code does and WHY it's implemented that way
- Dependencies and packages must be properly mentioned with exact version numbers
- Libraries must be standard, top-used, and widely adopted in the industry

### Code Structure with MCP
- Use context7 MCP for every code and setup-related task
- Context7 MCP is installed and SHOULD be used for installing any packages
- Docusaurus should fully utilize CONTEXT7 MCP documentation

## Build and Deployment Gates

### Pre-deployment Quality Checks
- Docusaurus must build successfully without any errors or warnings
- Broken link checker must pass (verify all internal and external links)
- Spell check must pass with strict verification
- All code examples must run successfully in test environment

## Governance

### Amendment Procedure
This constitution may only be amended through the following process: proposal submitted with justification, peer review by at least two qualified reviewers, trial period of 30 days if applicable, and formal approval documented in version history.

### Versioning Policy
Version numbers follow semantic versioning (MAJOR.MINOR.PATCH):
- MAJOR: Fundamental changes to core principles or governance
- MINOR: Addition of new principles or material expansion of guidance
- PATCH: Clarifications, wording adjustments, typo fixes

### Compliance Review
All contributions must verify compliance with these principles; Non-compliance must be documented with justification; Regular reviews ensure continued adherence to standards.

**Version**: 1.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10