# Data Model: Physical AI & Humanoid Robotics Book

## Content Entities

### Book
- **id**: string (unique identifier)
- **title**: string (main title of the book)
- **subtitle**: string (optional subtitle)
- **authors**: array of Author objects
- **phase**: enum (RESEARCH | FOUNDATION | ANALYSIS | SYNTHESIS)
- **description**: string (brief description)
- **tags**: array of strings (topic tags)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)
- **metadata**: object (additional metadata)

### Chapter
- **id**: string (unique identifier)
- **bookId**: string (reference to Book)
- **title**: string (chapter title)
- **subtitle**: string (optional subtitle)
- **content**: string (main content in Markdown)
- **phase**: enum (RESEARCH | FOUNDATION | ANALYSIS | SYNTHESIS)
- **order**: integer (position in book)
- **authors**: array of Author objects
- **tags**: array of strings (topic tags)
- **references**: array of Reference objects
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)
- **frontmatter**: object (Docusaurus frontmatter)

### Section
- **id**: string (unique identifier)
- **chapterId**: string (reference to Chapter)
- **title**: string (section title)
- **content**: string (section content in Markdown)
- **order**: integer (position in chapter)
- **type**: enum (TEXT | CODE | DIAGRAM | EXAMPLE | EXERCISE)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

### Reference (Citation)
- **id**: string (unique identifier)
- **type**: enum (BOOK | JOURNAL | WEB | CONFERENCE | OTHER)
- **title**: string (title of the referenced work)
- **authors**: array of Author objects
- **year**: integer (publication year)
- **journal**: string (journal name if applicable)
- **publisher**: string (publisher name)
- **url**: string (URL if applicable)
- **doi**: string (DOI if applicable)
- **isbn**: string (ISBN if applicable)
- **apaCitation**: string (APA formatted citation)
- **accessedDate**: datetime (date accessed for web sources)

### Author
- **id**: string (unique identifier)
- **name**: string (full name)
- **affiliation**: string (institution/organization)
- **email**: string (contact email)
- **orcid**: string (ORCID identifier if applicable)

### Tag
- **id**: string (unique identifier)
- **name**: string (tag name)
- **category**: string (category of tag)
- **description**: string (optional description)

## Validation Rules

### Book Validation
- Title must be 1-200 characters
- Must have at least one author
- Phase must be one of the defined enum values
- Status must be one of the defined enum values

### Chapter Validation
- Title must be 1-200 characters
- Must belong to a valid book
- Order must be a positive integer
- Content must be in valid Markdown format
- Phase must match parent book's phase or be more specific

### Section Validation
- Title must be 1-200 characters
- Must belong to a valid chapter
- Order must be a positive integer
- Type must be one of the defined enum values

### Reference Validation
- Must have a title
- Must have at least one author or organization
- APA citation must follow APA 7th edition format
- Year must be a valid year (1000-2100)

## State Transitions

### Book States
- DRAFT → REVIEW (when content is complete and ready for review)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

### Chapter States
- DRAFT → REVIEW (when content is complete and ready for review)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

### Section States
- DRAFT → REVIEW (when content is complete and ready for review)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

## Relationships
- Book has many Chapters
- Chapter belongs to one Book
- Chapter has many Sections
- Section belongs to one Chapter
- Chapter has many References (citations)
- Book has many Authors (through chapters)
- Content entities can have many Tags