# Contribution Guidelines

As an open-source project, we welcome and encourage the community to submit patches directly to
Kairos. In our collaborative open source environment, standards and methods for submitting
changes help reduce the chaos that can result from an active development community.

This document explains how to report bugs and request enhancements, and how to submit patches so
that they are accepted quickly in the codebase.

## Licensing

Licensing is very important to open source projects. It helps ensure the software continues to
be available under the terms that the author desired.

Kairos is licensed under the [Apache 2.0 license](LICENSE), and contributions are made under the
same license. Every source file starts with the Apache-2.0 header used across the repository.

A license tells you what rights you have as a developer, as provided by the copyright holder. It
is important that the contributor fully understands the licensing rights and agrees to them.
Sometimes the copyright holder isn't the contributor, such as when the contributor is doing work
on behalf of a company.

## Developer Certification of Origin (DCO)

To make a good faith effort to ensure licensing criteria are met, Kairos requires the Developer
Certificate of Origin (DCO) process to be followed.

The DCO is an attestation attached to every contribution made by every developer. In the commit
message of the contribution, the developer simply adds a `Signed-off-by` statement and thereby
agrees to the DCO.

When a developer submits a patch, it is a commitment that the contributor has the right to
submit the patch per the license. The DCO agreement is shown below and at
[http://developercertificate.org/](http://developercertificate.org/).

``` txt
Developer's Certificate of Origin 1.1

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the
    best of my knowledge, is covered under an appropriate open
    source license and I have the right under that license to
    submit that work with modifications, whether created in whole
    or in part by me, under the same open source license (unless
    I am permitted to submit under a different license), as
    Indicated in the file; or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including
    all personal information I submit with it, including my
    sign-off) is maintained indefinitely and may be redistributed
    consistent with this project or the open source license(s)
    involved.
```

### DCO Sign-Off Methods

The DCO requires that a sign-off message, in the following format, appears on each commit in the
pull request:

``` txt
Signed-off-by: Sofforus Jones <sjones@gmail.com>
```

The DCO text can either be manually added to your commit body, or you can add either `-s` or
`--signoff` to your usual Git commit commands. If you forget to add the sign-off you can also
amend a previous commit with the sign-off by running `git commit --amend -s`. If you've pushed
your changes to GitHub already you'll need to force push your branch after this with
`git push -f`.

Note: the name and email address of the account you use to submit your PR must match the name
and email address on the `Signed-off-by` line in your commit message.

## Reporting bugs and requesting features

Open an issue at [github.com/PlanSys2/kairos/issues](https://github.com/PlanSys2/kairos/issues).
For a bug, include:

- the domain and problem (or the smallest ones that show it), and the plan if it is about
  validation;
- the exact command or API call, and what you expected;
- the output with `-v`, or with `--debug=<channels>` for the stage that fails (for instance
  `--debug=ground,search`);
- the Kairos version (`kairos --version`) and your platform and compiler.

## Submitting patches

- One branch and one pull request per change, against `main`, with a clear description of what
  changes and why.
- Every change comes with its tests in the same pull request. Tests cover edge values and
  sequences, not just the happy path. Bugs get a regression test.
- Changes to the public API (`include/kairos/`) keep it free of internal types. A change to a
  public struct changes the ABI: bump the minor version while Kairos is 0.x, in
  `CMakeLists.txt`, `package.xml` and `pixi.toml` (CMake checks that the three agree).
- Planning changes that may affect coverage or speed are measured with `tools/coverage.py`
  against the previous version; say what changed in the pull request.

### Building and testing

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Before opening the pull request, check also:

- **Sanitizers**: `-DKAIROS_SANITIZE="address;undefined"` and `-DKAIROS_SANITIZE=thread`, in
  separate build directories.
- **Coverage**: CI fails if the line coverage of `src/` and `include/` drops below 90 %. To
  measure it locally, see [Testing](README.md#testing).
- **External corpus** (optional, slow): `tools/fetch_corpus.sh` and then
  `KAIROS_PDDL_CORPUS=~/.cache/kairos/corpus ctest --test-dir build`.

### Code style

ROS 2 style (2-space indent, braces on their own line for classes and functions, 100 columns),
checked by the `cpplint` and `uncrustify` tests (`ctest -L lint`). To fix the formatting:

```bash
uncrustify -c tools/lint/uncrustify.cfg --replace --no-backup <files>
```

Comments are short and say why, not what. Error messages say what failed, where (file, line and
column when there is one) and, when it helps, how to fix it.
