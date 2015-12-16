function serialRead()
clear
s = serial('COM3');
set(s, 'BaudRate', 250000, 'Timeout', 0.002, 'ByteOrder', 'littleEndian');
fopen(s);

finishup = onCleanup(@() cleanup(s));
    function cleanup(s)
        fclose(s);
        delete(s);
        clear s
%         whos
        disp(time);
        disp(i-1);
        timeVec = timeVec(1:i-1);
        pitchPsi = pitchPsi(1:i-1);
        pitchPhi = pitchPhi(1:i-1);
        pitchVa = pitchVa(1:i-1);
        rollPsi = rollPsi(1:i-1);
        rollPhi = rollPhi(1:i-1);
        rollVa = rollVa(1:i-1);
%         whos
        disp('Cleaned Up. Plotting Now.')
%         figure('OuterPosition',[800+1 40+1 800 900-40]);
        filename = strcat(datestr(clock,'yyyymmddTHHMMSS'), '.csv');
        csvwrite(filename, [timeVec pitchPsi pitchPhi pitchVa rollPsi rollPhi rollVa]);
        plot(timeVec, pitchPsi, '--', 'linewidth', 2);
        hold all
        plot(timeVec, pitchPhi, '--', 'linewidth', 2);
        plot(timeVec, pitchVa, '--', 'linewidth', 2);
        plot(timeVec, rollPsi, 'linewidth', 2);
        plot(timeVec, rollPhi, 'linewidth', 2);
        plot(timeVec, rollVa, 'linewidth', 2);
        legend('pitchPsi', 'pitchPhi', 'pitchVa', 'rollPsi', 'rollPhi', 'rollVa');
        xlabel('time [s]');
        ylabel('value [rad or rad/s or V]');
        %
        %         subplot(2,1,1);
        %         hold all
        %         plot(domain, diestaticdie(Vout, 201), 'LineWidth', 2);
        %         plot(domain, diestaticdie(Vset, 201), 'LineWidth', 2);
        %         %         plot(domain, diestaticdie(Vb, 201), 'LineWidth', 2);
        %         plot(domain, diestaticdie(Vcontrol, 201), 'LineWidth', 2);
        %         %         leg = legend('$V_{out}$', '$V_{set}$', '$V_b$', '$V_{control}$', 'location', 'best');
        %         leg = legend('$V_{out}$', '$V_{set}$', '$V_{control}$', 'location', 'best');
        %         set(leg, 'fontsize', 14, 'interpreter', 'latex');
        %         xlab = xlabel('$Time [s]$', 'FontSize', 22);
        %         ylab = ylabel('$Voltage [V]$', 'FontSize', 22);
        %         set(xlab, 'interpreter', 'latex'); set(ylab, 'interpreter', 'latex');
        %         set(gca, 'FontSize', 13);
        %         h = title('Voltage Control', 'FontSize', 17);
        %         set(h, 'interpreter', 'latex');
        %
        %         subplot(2,1,2);
        %         hold all
        %         plot(domain, diestaticdie(Tout, 201), 'LineWidth', 2);
        %         plot(domain, diestaticdie(Tset, 201), 'LineWidth', 2);
        %         leg = legend('$T_{out}$', '$T_{set}$', 'location', 'best');
        %         set(leg, 'fontsize', 14, 'interpreter', 'latex');
        %         xlab = xlabel('$Time [s]$', 'FontSize', 22);
        %         ylab = ylabel('$Temperature [^oC]$', 'FontSize', 22);
        %         set(xlab, 'interpreter', 'latex'); set(ylab, 'interpreter', 'latex');
        %         set(gca, 'FontSize', 13);
        %         h = title('Temperature Control', 'FontSize', 17);
        %         set(h, 'interpreter', 'latex');
        
        %         filtered = diestaticdie(Vout, 201);
        %         plot(domain, filtered);
        %         filtered = sgolayfilt(Vout, 3, 201);
        %         plot(domain, filtered);
    end

timeVec = zeros(100000, 1);
pitchPsi = zeros(100000, 1);
pitchPhi = zeros(100000, 1);
pitchVa = zeros(100000, 1);
rollPsi = zeros(100000, 1);
rollPhi = zeros(100000, 1);
rollVa = zeros(100000, 1);
% vals = zeros(100000, 6);
i = 1;
while(true)
    if(get(s, 'BytesAvailable') >= 1)
%         get(s, 'BytesAvailable')
%         try
            if (i == 1 || i == 2)
                tic
            else
                time = toc;
            end
            timedata = fread(s, 1, 'uint32')/1000;
%             timeVec(i) = timedata;
            valdata = fread(s, 6, 'float32')';
            if length(valdata) ~= 6
                continue
            end
%             vals(i,:) = valdata;
%             if abs(timedata-timeVec(i-1+(i==0))) > 10
%                 timedata = timeVec(i-1+(i==0)) + 0.010;
%             end
            timeVec(i) = timedata;
%             vals(or(vals<-100, vals>100)) = 0;
%             for j=1:6
%                 if abs(valdata(j)-vals(i-1+(i==0),j)) > 2
%                     valdata(j) = vals(i-1+(i==0),j);
%                 end
%             end
            pitchPsi(i) = valdata(1);
            pitchPhi(i) = valdata(2);
            pitchVa(i) = valdata(3);
            rollPsi(i) = valdata(4);
            rollPhi(i) = valdata(5);
            rollVa(i) = valdata(6);
            
            %             disp([timeVec(i); vals]');
            if mod(i, 100) == 0
                disp([timedata valdata]);
            end
            i = i + 1;
%         catch ME
%             disp(ME);
%         end
    else
        %         disp(['waiting ' num2str(i)]);
    end
end
end