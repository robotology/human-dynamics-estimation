###
# brewInstallOrUpgrade <pkg>
# gracefully handle case of installing something
# that's already installed
function brewInstallOrUpgrade {
	local aux=$(brew ls --versions "$1")
	echo "${aux}"
    if [[ ! -z "$aux" ]]; then
        HOMEBREW_NO_AUTO_UPDATE=1 brew upgrade "$1" || true
    else
        HOMEBREW_NO_AUTO_UPDATE=1 brew install "$1" 
    fi
}
