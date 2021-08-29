if $- != *i*; then
    return
fi

if command -v dircolors &> /dev/null; then
    eval "$(dircolors)"
fi

if command -v starship &> /dev/null; then
    eval "$(starship init bash)"
fi

alias grep="grep --color=auto"
alias ls="ls --color=auto"
alias sudo="sudo "

export EDITOR=vim
export PATH="$HOME/bin:$PATH"
export PS1="\w \$ "

shopt -s autocd
shopt -s cdspell
shopt -s globstar
shopt -s histappend
