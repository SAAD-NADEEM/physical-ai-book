# Quickstart Guide: Docusaurus-based Technical Book on Physical AI and Humanoid Robotics

## Overview

This quickstart guide provides steps to quickly set up, develop, and deploy the Docusaurus-based technical book on Physical AI and Humanoid Robotics. This guide is designed for developers and content creators who need to get the documentation site running quickly.

## Prerequisites

Before starting, ensure you have the following installed:

- **Node.js**: Version 18 or higher
- **npm or yarn**: Package managers that come with Node.js
- **Git**: Version control system
- **A GitHub account**: For deployment to GitHub Pages

## Step 1: Project Setup

1. **Clone or create the repository**
   ```bash
   git clone https://github.com/your-username/physical-ai-book.git
   cd physical-ai-book
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```
   or if using yarn:
   ```bash
   yarn install
   ```

3. **Verify installation**
   ```bash
   npm run start
   ```
   This command starts a local development server with hot reloading. The site will be accessible at http://localhost:3000.

## Step 2: Project Structure Overview

Familiarize yourself with the main directories:

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

## Step 3: Configuration

1. **Configure docusaurus.config.js**:
   - Update site metadata (title, tagline, url, baseUrl)
   - Configure GitHub Pages deployment settings (organizationName, projectName, deploymentBranch)
   - Set up theme and plugin configurations

2. **Set up navigation in sidebars.js**:
   - Organize modules and chapters hierarchically
   - Set sidebar positions for proper ordering
   - Ensure all content is properly linked

## Step 4: Adding Content

1. **Create a new chapter**:
   ```markdown
   ---
   title: Introduction to ROS 2
   description: Overview of ROS 2 architecture and concepts for humanoid robotics
   keywords: [ros2, robotics, middleware, architecture]
   sidebar_position: 1
   module_ref: module-1-ros2
   prerequisites: []
   learning_objectives: ["Understand ROS 2 architecture", "Identify core components"]
   estimated_reading_time: 30
   exercises_count: 3
   ---

   # Introduction to ROS 2

   ## Learning Objectives
   - Understand the architecture of ROS 2
   - Identify the core components of a ROS 2 system
   - Recognize the differences between ROS 1 and ROS 2

   ## Prerequisites
   - Basic understanding of robotics concepts
   - Programming experience in Python or C++

   ## Core Concepts
   ROS 2 (Robot Operating System 2) is a flexible framework for writing robot applications...

   ## Implementation
   Here's an example of a simple ROS 2 publisher node:

   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

   ## Exercises
   1. **Logical Exercise**: Explain the advantages of ROS 2 over ROS 1.
   2. **Conceptual Exercise**: Describe the role of DDS in ROS 2 architecture.
   3. **Implementation Exercise**: Create a ROS 2 package with a publisher and subscriber node.

   ## Summary
   This chapter introduced the ROS 2 framework, its architecture, and basic concepts...

   ## References
   - ROS 2 Documentation: https://docs.ros.org/
   - DDS Standard: https://www.omg.org/spec/DDS/
   ```
   ```

2. **Update sidebars.js** to include the new chapter:
   ```js
   module.exports = {
     tutorialSidebar: [
       {
         type: 'category',
         label: 'Module 1: ROS 2 Fundamentals',
         items: [
           'module-1-ros2/chapter-1',  // Add your new chapter here
           'module-1-ros2/chapter-2',
           // ... other chapters
         ],
       },
       // ... other modules
     ],
   };
   ```

## Step 5: Building and Testing

1. **Build the site**:
   ```bash
   npm run build
   ```
   This creates a static build in the `build/` directory.

2. **Test the build locally**:
   ```bash
   npm run serve
   ```
   This serves the built site at http://localhost:3000 for testing.

3. **Check for broken links**:
   ```bash
   npm run docusaurus check-links
   ```

4. **Run spell check** (if available):
   ```bash
   # You may need to install a spell check plugin
   npm install docusaurus-plugin-spell-checker
   ```

## Step 6: Deployment to GitHub Pages

1. **Configure GitHub Actions** in `.github/workflows/deploy.yml`:
   ```yaml
   name: Deploy to GitHub Pages
   on:
     push:
       branches: [main]
   
   jobs:
     deploy:
       name: Deploy to GitHub Pages
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v3
         - uses: actions/setup-node@v3
           with:
             node-version: 18
             cache: npm
   
         - name: Install dependencies
           run: npm install
         - name: Build website
           run: npm run build
   
         - name: Deploy to GitHub Pages
           uses: peaceiris/actions-gh-pages@v3
           with:
             github_token: ${{ secrets.GITHUB_TOKEN }}
             publish_dir: ./build
             publish_branch: gh-pages
   ```

2. **Push changes to GitHub**:
   ```bash
   git add .
   git commit -m "Add new chapter content"
   git push origin main
   ```

3. **Monitor the deployment** in the GitHub Actions tab.

4. **Visit your deployed site**: Usually available at `https://your-username.github.io/physical-ai-book`

## Common Commands

| Command | Description |
|--------|-------------|
| `npm run start` | Start local development server |
| `npm run build` | Build static site for production |
| `npm run serve` | Serve built site locally |
| `npm run docusaurus clear` | Clear Docusaurus cache |
| `npm run docusaurus swizzle <component>` | Customize a Docusaurus component |
| `npm run check-links` | Check for broken links |

## Troubleshooting

### Common Issues and Solutions:

1. **"Port 3000 is already in use"**
   - Solution: Use a different port: `npm run start -- --port 3001`

2. **"Command not found: docusaurus"**
   - Solution: Ensure you're in the website directory and dependencies are installed

3. **Build fails with "Module not found" errors**
   - Solution: Run `npm install` again to ensure all dependencies are present

4. **Images not showing up**
   - Solution: Ensure images are placed in the `static/` directory and referenced with the correct path

5. **Search not working**
   - Solution: Ensure Algolia DocSearch is properly configured in `docusaurus.config.js`

## Next Steps

After completing the quickstart:

1. Review the [Data Model](./data-model.md) document for the full content structure
2. Consult the [Implementation Plan](./plan.md) for the complete development workflow
3. Follow the constitutional principles for consistent content creation
4. Use the [Research Summary](./research.md) for best practices and implementation details