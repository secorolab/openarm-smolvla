# Build and publish the documentation

The documentation website includes the README, all Markdown guides in `docs/`,
and the C++ headers. New Markdown guides are included automatically.
Broken Doxygen references fail the build so links can be fixed before publishing.

## Local preview

Install CMake and Doxygen 1.16 or newer. CI uses Doxygen 1.16.1.
No ROS, CUDA, Python model environment or robot connection is required.
From the repository root:

```bash
cmake -S docs -B build/docs
cmake --build build/docs
python3 -m http.server 8000 --directory build/docs/site/html
```

Open http://localhost:8000. If Doxygen is outside PATH, add
`-DDOXYGEN_EXECUTABLE=/path/to/doxygen` to the CMake configuration command.

## GitHub Pages

1. Commit and push the documentation files and `.github/workflows/docs.yml`.
2. Open the repository's **Settings → Pages → Build and deployment** and set
   **Source** to **GitHub Actions**. Repository administration permission is needed.
3. Push to `main`, or run the **docs** workflow manually on `main` in **Actions**.
4. Open the published URL reported by the deployment job.

Pull requests build documentation without publishing. Only `main` deploys.
The expected default site address is https://secorolab.github.io/openarm-smolvla/;
it is not live until Pages is enabled and deployment succeeds.

See [GitHub's publishing-source instructions](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site).
