## Git Workflow Outline (C++ Project)

### Branch Structure
```
main
 └── staging
       └── module/staging
             └── module/dev/{branch-name}
```

- **main**: Production-ready, stable code.
- **staging**: Integration and release-testing branch.
- **module/staging**: Aggregates module-specific tested features.
- **module/dev/{branch-name}**: Individual developer feature branches.

### Folder Structure
```
src/
 └── qcar_{module}/
      ├── components/
      ├── utils/
      ├── tests/
      └── include/
```

### Workflow Steps
1. **Create Dev Branch**:
```bash
git checkout module/staging
git checkout -b module/dev/{branch-name}
```

2. **Develop & Commit**: Regular commits with clear, descriptive messages.

3. **Push & Open PR**:
```bash
git push origin module/dev/{branch-name}
```
- merge into `module/staging`.
- once you have a solution, Pull request into `staging`

4. **Code Review & Merge**:
- Merge `module/staging` PR after review into `staging`.
- You can keep or delete dev branches if you would like

5. **Integration**:
- Regularly merge `module/staging` into general `staging`.
- Run integration tests.

6. **Release**:
- Merge `staging` into `main` for deployment after QA approval.

### Benefits
- Clear separation of tasks.
- Simplified integration and debugging.
- Improved collaboration and code quality.
- Structured release management, reducing production risks.

