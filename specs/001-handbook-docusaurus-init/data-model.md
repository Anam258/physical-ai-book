# Data Model: Physical AI & Humanoid Robotics Handbook - Part 1 Focus

## Entity: Chapter

### Description
A `Chapter` represents a single section of the Physical AI & Humanoid Robotics Handbook. It is primarily a Markdown or MDX file containing educational content, code examples, and potentially embedded React components for interactivity. Chapters are organized sequentially and contribute to the overall structure and navigation of the handbook.

### Attributes

*   **`id`**: (Implicit) A unique identifier for the chapter, typically derived from its file path and name (e.g., `week-1-introduction`).
*   **`title`**: The human-readable title of the chapter, used for display in the sidebar and page headers (e.g., "Week 1: Introduction to Physical AI"). This is usually defined in the Markdown front matter (`title` field).
*   **`content`**: The main body of the chapter, written in Markdown/MDX format, including text, images, code blocks, and embedded components.
*   **`sidebar_position`**: (Optional) A numeric value in the Markdown front matter that determines the order of the chapter in the sidebar navigation. If not specified, alphabetical ordering applies.
*   **`slug`**: (Optional) A URL-friendly identifier for the chapter, also defined in front matter.
*   **`_category_.json` / `_category_.yml`**: (For categories/folders) Metadata files for organizing groups of chapters in the sidebar, allowing for custom labels, positions, and additional properties.

### Relationships

*   **Handbook (Container)**: Chapters are contained within the overall Docusaurus handbook structure. They are organized into logical groups (categories/folders) which form the hierarchy of the sidebar.
*   **Navigation**: Chapters are linked through the Docusaurus sidebar and potentially through internal links within chapter content.

### Validation Rules

*   Each chapter file (`.md` or `.mdx`) must have valid Markdown/MDX syntax.
*   Chapter front matter (if present) should be valid YAML.
*   `sidebar_position` (if used) should be a valid number.

