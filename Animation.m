for t = 1:length(j)
    clf
    plot(j(1:t),dK1(1:t),'-oc','LineWidth',3,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor','c','MarkerSize',3)
    xlim([0 inf])
    grid on
    ylabel('||K1^i-K1^*||')
    xlabel('Iteration step')
    legend('||K1^i-K1^*||')
    title(strcat("Iteration: ",num2str(t)))
    pause(0.2)
    
    % gif utilities
    set(gcf,'color','w'); % set figure background to white
    drawnow;
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    outfile = 'K1_off.gif';
 
    % On the first loop, create the file. In subsequent loops, append.
    if t==1
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'loopcount',inf);
    else
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'writemode','append');
    end
end

for t = 1:length(j)
    clf
    plot(j(1:t),dK2(1:t),'-oc','LineWidth',3,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor','c','MarkerSize',3)
    xlim([0 inf])
    grid on
    ylabel('||K2^i-K2^*||')
    xlabel('Iteration step')
    legend('||K2^i-K2^*||')
    title(strcat("Iteration: ",num2str(t)))
    pause(0.2)
    
    % gif utilities
    set(gcf,'color','w'); % set figure background to white
    drawnow;
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    outfile = 'K2_off.gif';
 
    % On the first loop, create the file. In subsequent loops, append.
    if t==1
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'loopcount',inf);
    else
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'writemode','append');
    end
end

for t = 1:length(j)
    clf
    plot(j(1:t),dK3(1:t),'-oc','LineWidth',3,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor','c','MarkerSize',3)
    xlim([0 inf])
    grid on
    ylabel('||K3^i-K3^*||')
    xlabel('Iteration step')
    legend('||K3^i-K3^*||')
    title(strcat("Iteration: ",num2str(t)))
    pause(0.2)
    
    % gif utilities
    set(gcf,'color','w'); % set figure background to white
    drawnow;
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    outfile = 'K3_off.gif';
 
    % On the first loop, create the file. In subsequent loops, append.
    if t==1
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'loopcount',inf);
    else
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'writemode','append');
    end
end

for t = 1:length(j)
    clf
    plot(j(1:t),dK(1:t),'-oc','LineWidth',3,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor','c','MarkerSize',3)
    xlim([0 inf])
    grid on
    ylabel('||K^i-K^*||')
    xlabel('Iteration step')
    legend('||K^i-K^*||')
    title(strcat("Iteration: ",num2str(t)))
    pause(0.2)
    
    % gif utilities
    set(gcf,'color','w'); % set figure background to white
    drawnow;
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    outfile = 'K_off.gif';
 
    % On the first loop, create the file. In subsequent loops, append.
    if t==1
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'loopcount',inf);
    else
        imwrite(imind,cm,outfile,'gif','DelayTime',0.2,'writemode','append');
    end
end
