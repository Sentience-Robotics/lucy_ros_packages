# shellcheck shell=zsh
# Caller must set: _SCRIPT_DIR="${0:A:h}"  immediately before sourcing this file.
# Exports LUCY_WORKSPACE and LUCY_CONTROL_PANEL_DIR.

export LUCY_WORKSPACE=""
if [[ -n "${LUCY_WS}" && -d "${LUCY_WS}" ]]; then
  export LUCY_WORKSPACE="${LUCY_WS:A}"
elif [[ "${_SCRIPT_DIR}" == */lib/lucy_bringup ]]; then
  if _lw="$(builtin cd "${_SCRIPT_DIR}/../../.." 2>/dev/null && pwd)" && [[ -n "${_lw}" ]]; then
    export LUCY_WORKSPACE="${_lw}"
  fi
  unset _lw
elif [[ "${_SCRIPT_DIR}" == */lucy_bringup/system_scripts ]]; then
  if _lw="$(builtin cd "${_SCRIPT_DIR}/../../../.." 2>/dev/null && pwd)" && [[ -n "${_lw}" ]]; then
    export LUCY_WORKSPACE="${_lw}"
  fi
  unset _lw
fi

[[ -z "${LUCY_WORKSPACE}" ]] && export LUCY_WORKSPACE="${HOME}/lucy_ws"

export LUCY_CONTROL_PANEL_DIR="${LUCY_WORKSPACE}/src/lucy_control_panel"
unset _SCRIPT_DIR
