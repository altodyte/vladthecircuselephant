function serialRead()
clear
s = serial('COM3');
set(s, 'BaudRate', 250000, 'Timeout', 0.005, 'ByteOrder', 'littleEndian');
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
        rollPhi = rollPhi(1:i-1);
        rollPsi = rollPsi(1:i-1);
        rollVa = rollVa(1:i-1);
%         whos
        disp('Cleaned Up. Plotting Now.')
%         figure('OuterPosition',[800+1 40+1 800 900-40]);
        filename = strcat(datestr(clock,'yyyymmddTHHMMSS'), '.csv');
        csvwrite(filename, [timeVec pitchPsi pitchPhi pitchVa rollPsi rollPhi rollVa]);
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
rollPhi = zeros(100000, 1);
rollPsi = zeros(100000, 1);
rollVa = zeros(100000, 1);
i = 1;
while(true)
    if(get(s, 'BytesAvailable') >= 1)
%         get(s, 'BytesAvailable')
        try
            if (i == 1 || i == 2)
                tic
            else
                time = toc;
            end
            timeVec(i) = fread(s, 1, 'uint32')/1000;
            vals = fread(s, 6, 'float32');
            pitchPsi(i) = vals(1);
            pitchPhi(i) = vals(2);
            pitchVa(i) = vals(3);
            rollPhi(i) = vals(4);
            rollPsi(i) = vals(5);
            rollVa(i) = vals(6);
            
%             disp([timeVec(i); vals]');
            %             if mod(i,200) == 0
            %                 disp([timeVec(i); vals]');
            %             end
            i = i + 1;
        catch ME
            disp(ME);
        end
    else
        %         disp(['waiting ' num2str(i)]);
    end
end
end