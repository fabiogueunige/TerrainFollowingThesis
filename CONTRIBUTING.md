# Contributing to Terrain Following Thesis

Thank you for your interest in contributing to this project! This document provides guidelines for contributing to the AUV Terrain Following system.

## 📋 Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Testing Guidelines](#testing-guidelines)
- [Documentation](#documentation)
- [Commit Messages](#commit-messages)

---

## 🤝 Code of Conduct

This is an academic research project. We expect all contributors to:

- Be respectful and constructive
- Provide clear explanations for changes
- Cite sources and references appropriately
- Acknowledge the work of others
- Focus on technical merit

---

## 🚀 Getting Started

### Prerequisites

1. **MATLAB R2020b or later**
2. **Git** for version control
3. **Basic understanding** of:
   - Kalman Filtering
   - Control Systems
   - Marine Robotics (helpful but not required)

### Setup

```bash
# Fork the repository on GitHub
# Clone your fork
git clone https://github.com/YOUR_USERNAME/TFThesis.git
cd TFThesis

# Add upstream remote
git remote add upstream https://github.com/fabiogueunige/TFThesis.git

# Create a development branch
git checkout -b feature/your-feature-name
```

---

## 💡 How to Contribute

### Areas for Contribution

We welcome contributions in the following areas:

#### 1. **Bug Fixes**
- Fix calculation errors
- Resolve edge cases
- Improve error handling

#### 2. **New Features**
- Additional sensor models
- Alternative estimation algorithms (UKF, PF)
- Enhanced visualization tools
- Performance optimizations

#### 3. **Documentation**
- Improve code comments
- Add examples and tutorials
- Translate documentation
- Create diagrams and illustrations

#### 4. **Testing**
- Add unit tests
- Create test scenarios
- Validate against real data
- Performance benchmarks

#### 5. **Integration**
- ROS/ROS2 connectivity
- Simulink models
- Hardware interfaces
- External libraries

---

## 🔄 Development Workflow

### 1. Create an Issue

Before starting work, create an issue describing:
- The problem or feature
- Proposed solution
- Expected impact

### 2. Develop Your Changes

```bash
# Make sure you're on your feature branch
git checkout feature/your-feature-name

# Make changes
# Test thoroughly

# Commit your changes (see commit message guidelines)
git add .
git commit -m "feat: Add new feature description"
```

### 3. Keep Your Branch Updated

```bash
# Fetch upstream changes
git fetch upstream

# Rebase on master
git rebase upstream/master

# If conflicts, resolve them and continue
git rebase --continue
```

### 4. Submit a Pull Request

1. Push to your fork
   ```bash
   git push origin feature/your-feature-name
   ```

2. Open a Pull Request on GitHub
3. Fill out the PR template
4. Wait for review

### 5. Address Review Comments

- Be responsive to feedback
- Make requested changes
- Update the PR

---

## 📝 Coding Standards

### MATLAB Code Style

#### File Organization

```matlab
% File: example_function.m
% Description: Brief description of what the function does
%
% Inputs:
%   param1 - Description [units]
%   param2 - Description [units]
%
% Outputs:
%   output1 - Description [units]
%
% Example:
%   result = example_function(10, 20);
%
% Author: Your Name
% Date: YYYY-MM-DD

function output = example_function(param1, param2)
    % Implementation
end
```

#### Naming Conventions

```matlab
% Variables: lowercase with underscores
robot_position = [0, 0, 0];

% Constants: UPPERCASE
MAX_ALTITUDE = 10;

% Functions: camelCase or snake_case (be consistent)
function result = computeKalmanGain(P, H, R)
    % ...
end

% Matrices: uppercase for standard notation
R = eye(3);  % Rotation matrix
K = P * H' / S;  % Kalman gain
```

#### Comments

```matlab
% Use comments to explain WHY, not WHAT
% Good:
alpha_max = pi/4;  % Limit to ±45° to avoid gimbal lock

% Avoid:
alpha_max = pi/4;  % Set alpha_max to pi/4
```

#### Code Structure

```matlab
% Use clear section breaks
%% Initialization
% ...

%% Main Loop
for k = 1:N
    %% Prediction
    % ...
    
    %% Update
    % ...
end

%% Post-processing
% ...
```

### General Principles

1. **Readability First**
   - Clear variable names
   - Logical structure
   - Appropriate comments

2. **Modularity**
   - Functions should do one thing well
   - Avoid deeply nested code
   - Keep functions under 100 lines when possible

3. **Efficiency**
   - Preallocate arrays
   - Vectorize when possible
   - Avoid unnecessary loops

4. **Robustness**
   - Check input validity
   - Handle edge cases
   - Use meaningful error messages

---

## 🧪 Testing Guidelines

### Before Submitting

Test your changes with:

#### 1. Basic Functionality
```matlab
% Run main simulation
main_6DOF_3D

% Check for errors
% Verify outputs make sense
```

#### 2. Edge Cases
```matlab
% Test with extreme parameters
h_ref(:) = 1;  % Very low altitude
angle_range = [-pi/3, pi/3];  % Steep terrain

main_6DOF_3D
```

#### 3. Different Scenarios
```matlab
% Flat terrain
angle_range = [0, 0];

% Rolling terrain
angle_range = [-pi/6, pi/6];

% Test each scenario
```

#### 4. Performance
```matlab
% Check computation time
tic;
main_6DOF_3D;
elapsed = toc;
fprintf('Simulation time: %.2f seconds\n', elapsed);
```

### Test Documentation

Document your tests:

```matlab
%% Test: Altitude Tracking on Flat Terrain
% Expected: RMSE < 0.2m, no oscillations
% Result: RMSE = 0.15m ✓
```

---

## 📚 Documentation

### Code Documentation

When adding new functions:

```matlab
%% FUNCTION_NAME - Brief description
%
% Syntax:
%   output = function_name(input1, input2)
%   [output1, output2] = function_name(input1, input2, param)
%
% Description:
%   Detailed description of what the function does, including
%   any important algorithms or references.
%
% Inputs:
%   input1  - Description [units, type, size]
%   input2  - Description [units, type, size]
%   param   - (Optional) Description [default: value]
%
% Outputs:
%   output1 - Description [units, type, size]
%   output2 - Description [units, type, size]
%
% Example:
%   % Example usage
%   x = [1, 2, 3];
%   result = function_name(x, 10);
%
% See also: RELATED_FUNCTION1, RELATED_FUNCTION2
%
% References:
%   [1] Author, "Title", Journal, Year
%
% Author: Your Name
% Email: your.email@example.com
% Date: YYYY-MM-DD
% Version: 1.0
```

### README Updates

When adding features, update:
- Main README.md (if user-facing)
- Relevant documentation in `doc/`
- DEVELOPMENT_NOTES.md (if internal)

---

## 💬 Commit Messages

### Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Types

- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation changes
- **style**: Formatting, missing semicolons, etc.
- **refactor**: Code restructuring
- **perf**: Performance improvements
- **test**: Adding tests
- **chore**: Maintenance tasks

### Examples

```bash
# Simple fix
git commit -m "fix(ekf): Correct Jacobian calculation for h measurement"

# Feature with body
git commit -m "feat(sensors): Add DVL noise model

Implements realistic Doppler Velocity Log noise based on 
manufacturer specifications. Includes:
- Gaussian noise model
- Velocity-dependent error
- Configuration parameters

Refs #15"

# Breaking change
git commit -m "refactor(control)!: Change PID interface

BREAKING CHANGE: PID controller now requires explicit time step.
Update all calls to input_control() to include Ts parameter."
```

---

## 🔍 Code Review Process

### What Reviewers Look For

1. **Correctness**
   - Does it work as intended?
   - Are calculations correct?
   - Edge cases handled?

2. **Code Quality**
   - Readable and maintainable?
   - Follows style guide?
   - Well documented?

3. **Testing**
   - Adequate testing?
   - Performance acceptable?
   - No regressions?

4. **Documentation**
   - Changes documented?
   - Examples provided?
   - README updated?

### Review Checklist

- [ ] Code runs without errors
- [ ] Tests pass
- [ ] Documentation updated
- [ ] Follows coding standards
- [ ] Commit messages clear
- [ ] No unnecessary files included
- [ ] Performance acceptable

---

## 🏆 Recognition

Contributors will be acknowledged in:
- CONTRIBUTORS.md file
- Thesis acknowledgments (if significant contribution)
- Git commit history

---

## 📞 Questions?

If you have questions:

1. Check existing [documentation](TF_6DOF/doc/)
2. Search [issues](https://github.com/fabiogueunige/TFThesis/issues)
3. Create a new issue
4. Email: fabio.guelfi@libero.it

---

## 📜 License

By contributing, you agree that your contributions will be licensed under the same MIT License that covers the project.

---

**Thank you for contributing to advancing AUV navigation research!** 🤖🌊

---

*Last Updated: October 23, 2025*
