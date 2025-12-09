# Data Model: Docusaurus Design Implementation

## Color Variables

### Primary Colors
- **electric-blue**: #00F5FF (RGB: 0, 245, 255)
  - Usage: Main accent color, links, interactive elements
  - Accessibility: High contrast against dark backgrounds
- **electric-purple**: #8A2BE2 (RGB: 138, 43, 226)
  - Usage: Secondary accent, gradients, highlights
  - Accessibility: Complementary to electric blue
- **deep-navy**: #0A0E2A (RGB: 10, 14, 42)
  - Usage: Main background color
  - Accessibility: Dark theme foundation with good contrast

### Secondary Colors
- **glass-bg**: rgba(10, 14, 42, 0.7) (RGB: 10, 14, 42, 0.7)
  - Usage: Glassmorphism backgrounds for navbar/sidebar
  - Accessibility: Maintains contrast while providing transparency
- **text-primary**: #FFFFFF (RGB: 255, 255, 255)
  - Usage: Main text content
  - Accessibility: High contrast against dark backgrounds
- **text-secondary**: #B0B0B0 (RGB: 176, 176, 176)
  - Usage: Secondary text, muted content
  - Accessibility: Sufficient contrast while being less prominent

## Component Styles

### Glassmorphism Elements
- **backdrop-blur**: 10px
- **background**: rgba(10, 14, 42, 0.7)
- **border**: 1px solid rgba(0, 245, 255, 0.1)
- **border-radius**: 16px
- **box-shadow**: 0 8px 32px rgba(0, 245, 255, 0.1)

### Interactive Elements (Cards, Buttons)
- **transition**: transform 0.3s ease, box-shadow 0.3s ease
- **hover-transform**: translateY(-5px)
- **hover-shadow**: 0 10px 20px rgba(0, 245, 255, 0.3)
- **active-transform**: translateY(-2px)

### Typography
- **font-family**: System default (Docusaurus default) with fallbacks
- **heading-gradient**: linear-gradient(90deg, #00F5FF, #8A2BE2)
- **font-weights**: Regular (400), Medium (500), Bold (700)
- **line-height**: 1.6 for body text, 1.3 for headings

## Layout Components

### Feature Grid Structure
- **grid-columns**: Responsive (1 on mobile, 2 on tablet, 3 on desktop)
- **grid-gap**: 24px
- **card-padding**: 24px
- **card-radius**: 12px
- **card-background**: rgba(10, 14, 42, 0.5)

### Navigation Elements
- **navbar-height**: 60px
- **sidebar-width**: 280px (desktop), 100% (mobile)
- **glass-radius**: 16px
- **blur-amount**: 10px

## Animation Properties
- **transition-duration**: 0.3s
- **transition-function**: ease
- **hover-delay**: 0s (immediate response)
- **animation-timing**: smooth and subtle to maintain performance

## Responsive Breakpoints
- **mobile**: max-width: 768px
- **tablet**: 769px - 1024px
- **desktop**: min-width: 1025px

## Accessibility Properties
- **focus-outline**: 2px solid #00F5FF
- **contrast-ratio**: Minimum 4.5:1 for normal text, 3:1 for large text
- **reduced-motion**: Respects user preference for reduced motion
- **focusable-elements**: All interactive elements are keyboard accessible