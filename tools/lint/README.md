# Lint tools

`ctest -L lint` runs both checks.

| File | Origin | License |
|---|---|---|
| `cpplint.py` | Copy of the cpplint bundled in ROS 2 `ament_cpplint` (upstream https://github.com/cpplint/cpplint) | BSD-3-Clause (header kept) |
| `uncrustify.cfg` | Copy of `ament_code_style_0_78.cfg` from ROS 2 `ament_uncrustify` | Apache-2.0 |
| `kairos_cpplint.py` | Runs `cpplint.py` with the `ament_cpplint` filters and `KAIROS__` header guards | Apache-2.0 |

Fix formatting in place with:

```bash
uncrustify -c tools/lint/uncrustify.cfg --replace --no-backup <files>
```
