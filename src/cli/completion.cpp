// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "completion.hpp"

#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace kairos::cli
{

namespace
{

std::string join(const std::vector<std::string> & words, const char * sep = " ")
{
  std::string out;
  for (const auto & w : words) {
    out += (out.empty() ? "" : sep) + w;
  }
  return out;
}

const std::vector<std::string> & options_of(const CompletionSpec & spec, const std::string & c)
{
  static const std::vector<std::string> none;
  auto it = spec.options.find(c);
  return it == spec.options.end() ? none : it->second;
}

// Self-contained (no bash-completion package needed); zsh runs it through bashcompinit
std::string bash(const CompletionSpec & spec)
{
  std::ostringstream s;
  s << "# kairos completion for bash: eval \"$(kairos --completion bash)\"\n"
    << "_kairos()\n{\n"
    << "  local cur=\"${COMP_WORDS[COMP_CWORD]}\" prev=\"\"\n"
    << "  (( COMP_CWORD > 0 )) && prev=\"${COMP_WORDS[COMP_CWORD-1]}\"\n"
    << "  COMPREPLY=()\n"
    // "--debug=se" arrives as "--debug" "=" "se" ("=" breaks words)
    << "  local opt=\"$prev\" eq=0 pre=\"\"\n"
    << "  if [[ \"$cur\" == \"=\" ]]; then\n"
    << "    cur=\"\"; eq=1\n"
    << "  elif [[ \"$prev\" == \"=\" ]] && (( COMP_CWORD > 1 )); then\n"
    << "    opt=\"${COMP_WORDS[COMP_CWORD-2]}\"; eq=1\n"
    // Without "=" in COMP_WORDBREAKS the word keeps "--option=" in front
    << "  elif [[ \"$cur\" == --*=* ]]; then\n"
    << "    opt=\"${cur%%=*}\"; pre=\"$opt=\"; cur=\"${cur#*=}\"; eq=1\n"
    << "  fi\n"
    << "  case \"$opt\" in\n";
  for (const auto & [option, words] : spec.values) {
    s << "    " << option << ")\n"
      << "      COMPREPLY=($(compgen -W \"" << join(words) << "\" -P \"$pre\" -- \"$cur\"));"
      << " return 0;;\n";
  }
  for (const auto & [option, words] : spec.lists) {
    // Only after "=": the value of these options is optional
    s << "    " << option << ")\n"
      << "      if (( eq )); then\n"
      << "        local done=\"\"\n"
      << "        [[ \"$cur\" == *,* ]] && done=\"${cur%,*},\"\n"
      << "        COMPREPLY=($(compgen -W \"" << join(words) << "\" -P \"$pre$done\" -- "
      << "\"${cur##*,}\"))\n"
      << "        compopt -o nospace 2>/dev/null\n"
      << "        return 0\n"
      << "      fi;;\n";
  }
  if (!spec.file_options.empty()) {
    s << "    " << join(spec.file_options, "|") << ")\n"
      << "      compopt -o filenames 2>/dev/null\n"
      << "      COMPREPLY=($(compgen -f -- \"$cur\")); return 0;;\n";
  }
  if (!spec.dir_options.empty()) {
    s << "    " << join(spec.dir_options, "|") << ")\n"
      << "      compopt -o filenames 2>/dev/null\n"
      << "      COMPREPLY=($(compgen -d -- \"$cur\")); return 0;;\n";
  }
  if (!spec.number_options.empty()) {
    s << "    " << join(spec.number_options, "|") << ")\n"
      << "      return 0;;\n";
  }
  s << "  esac\n"
    << "  local command=\"\"\n"
    << "  (( COMP_CWORD > 1 )) && command=\"${COMP_WORDS[1]}\"\n"
    << "  case \"$command\" in\n"
    << "    " << join(spec.commands, "|") << ") ;;\n"
    << "    *) command=\"" << spec.commands.front() << "\";;\n"
    << "  esac\n"
    << "  if [[ \"$cur\" == -* ]]; then\n"
    << "    local options=\"\"\n"
    << "    (( COMP_CWORD == 1 )) && options=\"" << join(options_of(spec, "")) << "\"\n"
    << "    case \"$command\" in\n";
  for (const auto & c : spec.commands) {
    s << "      " << c << ") options=\"$options " << join(options_of(spec, c)) << "\";;\n";
  }
  s << "    esac\n"
    << "    COMPREPLY=($(compgen -W \"$options\" -- \"$cur\"))\n"
    << "    [[ ${#COMPREPLY[@]} -eq 1 && \"${COMPREPLY[0]}\" == *= ]] && "
    << "compopt -o nospace 2>/dev/null\n"
    << "    return 0\n"
    << "  fi\n"
    << "  (( COMP_CWORD == 1 )) && COMPREPLY=($(compgen -W \"" << join(spec.commands)
    << "\" -- \"$cur\"))\n"
    // PDDL files and directories first; any file if there is none
    << "  compopt -o filenames 2>/dev/null\n"
    << "  local IFS=$'\\n'\n"
    << "  local files=($(compgen -f -X '!*.pddl' -- \"$cur\") $(compgen -d -- \"$cur\"))\n"
    << "  [[ ${#files[@]} -eq 0 ]] && files=($(compgen -f -- \"$cur\"))\n"
    << "  COMPREPLY+=(\"${files[@]}\")\n"
    << "  return 0\n"
    << "}\n"
    << "complete -F _kairos kairos\n";
  return s.str();
}

std::string zsh(const CompletionSpec & spec)
{
  // Workspaces are sourced by scripts too: compinit only in interactive shells
  return "# kairos completion for zsh: eval \"$(kairos --completion zsh)\"\n"
         "[[ -o interactive ]] || return 0\n"
         "autoload -U +X compinit && { (( $+functions[compdef] )) || compinit; }\n"
         "autoload -U +X bashcompinit && bashcompinit\n" + bash(spec);
}

std::string fish(const CompletionSpec & spec)
{
  std::ostringstream s;
  s << "# kairos completion for fish: kairos --completion fish | source\n"
    << "complete -c kairos -e\n"
    << "complete -c kairos -n '__fish_use_subcommand' -a '" << join(spec.commands) << "'\n";
  auto contains = [](const std::vector<std::string> & v, const std::string & x) {
      for (const auto & y : v) {
        if (y == x) {
          return true;
        }
      }
      return false;
    };
  for (const auto & [command, options] : spec.options) {
    std::string when;
    if (command.empty()) {
      when = "__fish_use_subcommand";
    } else if (command == spec.commands.front()) {
      // The default command: also without naming it
      std::vector<std::string> others(spec.commands.begin() + 1, spec.commands.end());
      when = "not __fish_seen_subcommand_from " + join(others);
    } else {
      when = "__fish_seen_subcommand_from " + command;
    }
    for (std::string o : options) {
      bool eq = !o.empty() && o.back() == '=';
      if (eq) {
        o.pop_back();
      }
      s << "complete -c kairos -n '" << when << "' ";
      if (o.size() == 2) {
        s << "-s " << o.substr(1);
      } else {
        s << "-l " << o.substr(2);
      }
      if (auto v = spec.values.find(o); v != spec.values.end()) {
        s << " -x -a '" << join(v->second) << "'";
      } else if (auto l = spec.lists.find(o); l != spec.lists.end()) {
        s << " -a '" << join(l->second) << "'";
      } else if (contains(spec.file_options, o)) {
        s << " -r -F";
      } else if (contains(spec.dir_options, o)) {
        s << " -x -a '(__fish_complete_directories)'";
      } else if (contains(spec.number_options, o)) {
        s << " -x";
      }
      s << "\n";
    }
  }
  return s.str();
}

}  // namespace

std::optional<std::string> completion_script(std::string_view shell, const CompletionSpec & spec)
{
  if (shell == "bash") {
    return bash(spec);
  }
  if (shell == "zsh") {
    return zsh(spec);
  }
  if (shell == "fish") {
    return fish(spec);
  }
  return std::nullopt;
}

}  // namespace kairos::cli
