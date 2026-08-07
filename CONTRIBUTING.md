# Contributing

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/) so release-plz
can select the next version and generate the changelog. Mark incompatible API
changes with `!` and a `BREAKING CHANGE:` footer. For example:

```text
feat!: replace the projection API

BREAKING CHANGE: Callers must use the new projection method.
```

The release workflow runs `cargo-semver-checks`, but explicit breaking markers
are still required for changes—such as dependency type identity changes—that an
API comparison may not detect.

## Release setup

The `release` GitHub environment should be restricted to the `main` branch.
Configure the crate's crates.io trusted publisher with:

- GitHub owner: `strawlab`
- GitHub repository: `extended-unified-camera-model`
- Workflow filename: `release-plz.yml`
- Environment name: `release`

Allow GitHub Actions to create and approve pull requests in the repository's
Actions settings. If branch protection prevents the default token from updating
release pull requests, add a suitable token as the `RELEASE_PLZ_TOKEN`
repository secret.
