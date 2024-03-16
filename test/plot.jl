using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
import PyPlot as plt

function plot1(X, Y; label="", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12, fig="")
    if fig != ""
        plt.figure(fig)
    end
    p = plt.plot(X, Y; label)
    plt.grid(true)
    p
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
end

function plot2(X, Y1, Y2; labels=["", ""], fig="", title="")
    fig_ = plt.figure(fig, figsize=(8, 6))
    ax1 = plt.subplot(211) 
    plt.suptitle(title, fontsize=14) # Super title
    plt.plot(X, Y1, label=labels[1]); 
    plt.ylabel(labels[1], fontsize=14);        
    plt.grid(true)
    plt.setp(ax1.get_xticklabels(), visible=false)
    ax2 = plt.subplot(212, sharex = ax1)
    plt.plot(X, Y2, label=labels[2])
    plt.grid(true)
    plt.ylabel(labels[2], fontsize=14);    
    plt.grid(true)
    plt.xlabel("time [s]", fontsize=14)
    plt.xlim(0, X[end])
    plt.tight_layout()
end

function plot3(X, Y1, Y2, Y3; labels=["", "", ""], fig="", title="")
    fig_ = plt.figure(fig, figsize=(8, 6))
    ax1 = plt.subplot(311) 
    plt.suptitle(title, fontsize=14) # Super title
    plt.plot(X, Y1, label=labels[1]); 
    plt.ylabel(labels[1], fontsize=14);          
    plt.grid(true)
    plt.setp(ax1.get_xticklabels(), visible=false)
    ax2 = plt.subplot(312, sharex = ax1)
    plt.plot(X, Y2, label=labels[2])
    plt.grid(true)
    plt.ylabel(labels[2], fontsize=14);    
    plt.grid(true)
    plt.setp(ax2.get_xticklabels(), visible=false)
    ax3 = plt.subplot(313, sharex = ax1)
    plt.plot(X, Y3, label=labels[3])
    plt.grid(true)
    plt.ylabel(labels[3], fontsize=14);    
    plt.grid(true)
    plt.xlabel("time [s]", fontsize=14)
    plt.xlim(0, X[end])
    plt.tight_layout()
end

function plot4(X, Y1, Y2, Y3, Y4; labels=["", "", "", ""], fig="", title="")
    fig_ = plt.figure(fig, figsize=(8, 6))
    ax1 = plt.subplot(311) 
    plt.suptitle(title, fontsize=14) # Super title
    plt.plot(X, Y1, label=labels[1]); 
    ylabel(labels[1], fontsize=14);          
    plt.grid(true)
    plt.setp(ax1.get_xticklabels(), visible=false)
    ax2 = plt.subplot(312, sharex = ax1)
    plt.plot(X, Y2, label=labels[2])
    plt.grid(true)
    ylabel(labels[2], fontsize=14);    grid(true)
    plt.setp(ax2.get_xticklabels(), visible=false)
    ax3 = plt.subplot(313, sharex = ax1)
    plt.plot(X, Y3, label=labels[3])
    plt.grid(true)
    plt.ylabel(labels[3], fontsize=14);    
    plt.grid(true)
    ax4 = plt.subplot(313, sharex = ax1)
    plt.plot(X, Y4, label=labels[4])
    plt.grid(true)
    plt.ylabel(labels[4], fontsize=14);    
    plt.grid(true)
    plt.xlabel("time [s]", fontsize=14)
    plt.xlim(0, X[end])
    plt.tight_layout()
end