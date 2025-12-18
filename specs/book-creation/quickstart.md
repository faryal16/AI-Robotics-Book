# Quickstart: Physical AI & Humanoid Robotics Book

## Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Python 3.11+ (for AI tools)
- Git for version control

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
# Navigate to website directory
cd website
npm install
```

### 3. Install AI Tools (if separate)
```bash
cd ai-tools
# Install Python dependencies
pip install -r requirements.txt
```

## Running the Development Server

### Start Docusaurus Development Server
```bash
cd website
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Creating Book Content

### 1. Create a New Chapter
Create a new Markdown file in the appropriate chapter directory:
```text
website/docs/
├── chapter-1/
│   ├── 1.1-introduction.md
│   ├── 1.2-section-name.md
│   └── 1.conclusion.md
├── chapter-2/
│   ├── 2.1-introduction.md
│   ├── 2.2-section-name.md
│   └── 2.conclusion.md
├── chapter-3/
│   ├── 3.1-introduction.md
│   ├── 3.2-section-name.md
│   └── 3.conclusion.md
└── additional chapters...
```

### 2. Chapter Frontmatter
Each chapter should include proper frontmatter:
```markdown
---
title: Chapter Title
description: Brief description of the chapter
tags: [tag1, tag2, tag3]
sidebar_position: 1
---

# Chapter Title

Content here...
```

### 3. Add Citations
Use APA format for citations and include them in the references section:
```markdown
According to Smith et al. (2023), this concept is fundamental to the field [1].

## References

1. Smith, J., Johnson, M., & Williams, K. (2023). *Important research paper*. Journal of Research, 45(2), 123-145. https://doi.org/10.1234/example
```

## Using AI Tools for Content Generation

### 1. Run Research Agent
```bash
cd ai-tools/research-agent
python research_agent.py --topic "your-topic" --output-path "../website/docs/phase/chapter.md"
```

### 2. Validate Citations
```bash
cd ai-tools/citation-checker
python validate_citations.py --path ../website/docs/phase/chapter.md
```

### 3. Validate Content
```bash
cd ai-tools/content-validator
python validate_content.py --path ../website/docs/phase/chapter.md
```

## Building for Production

### Build Static Site
```bash
cd website
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Serve Production Build Locally
```bash
npm run serve
```

## Deployment

### Deploy to GitHub Pages
```bash
npm run deploy
```

### Deploy to Other Platforms
Refer to Docusaurus deployment documentation for other hosting options (Netlify, Vercel, etc.).

## Development Workflow

1. Create new content in the appropriate phase directory
2. Use AI tools to assist with research and content generation
3. Validate citations and content
4. Test locally using `npm start`
5. Commit changes with descriptive messages
6. Submit pull request for review

## Useful Commands

- `npm start` - Start local development server
- `npm run build` - Build static site
- `npm run serve` - Serve production build locally
- `npm run deploy` - Deploy to GitHub Pages
- `npm run docusaurus -- --help` - Show all Docusaurus commands