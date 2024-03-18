using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
import PyPlot as plt

function plot1(X, Y; label="", fig="")
    if fig != ""
        plt.figure(fig)
    end
    plt.plot(X, Y; label)
    plt.grid(true)
    nothing
end

function plotxy(X, Y; xlabel="", ylabel, fig="")
    if fig != ""
        plt.figure(fig)
    end
    p = plt.plot(X, Y)
    plt.xlabel(xlabel, fontsize=14);
    plt.ylabel(ylabel, fontsize=14);  
    plt.grid(true)
    plt.tight_layout()
    nothing
end

function plotx(X, Y...; labels=nothing, fig="", title="")
    len=length(Y)
    fig_ = plt.figure(fig, figsize=(8, len*2))
    i=1
    ax=[]
    for y in Y
        subplot=100len+10+i
        if i==1
            push!(ax, plt.subplot(subplot))
        else
            push!(ax, plt.subplot(subplot, sharex=ax[1]))
        end
        if i==1
            plt.suptitle(title, fontsize=14) # Super title
        end
        if ! isnothing(labels)
            lbl=labels[i]
        else
            lbl=""
        end
        plt.plot(X, y, label=lbl)
        plt.ylabel(lbl, fontsize=14);  
        plt.grid(true)
        if i < len
            plt.setp(ax[i].get_xticklabels(), visible=false)
        end
        i+=1
    end
    plt.xlabel("time [s]", fontsize=14)
    plt.xlim(0, X[end])
    plt.tight_layout()
    nothing
end
