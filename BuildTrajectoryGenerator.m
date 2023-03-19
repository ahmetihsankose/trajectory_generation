% Function for the design of minimum-time multi-segment trajectories with 
% kinematic and frequency constraints
%
% The working principle is described in the paper (published in Control Engineering Practice, Vol. 87, 2019):
% "Trajectory Generation via FIR Filters: a Procedure for Time-Optimization 
% under Kinematic and Frequency Constraints"
% by  Luigi Biagiotti (luigi.biagiotti@unimore.it) 
% and Claudio Melchiorri (claudio.melchiorri@unibo.it)
%
% USAGE: BuildTrajectoryGenerator( KinematicConstraints, AngularFrequencies, SamplingTime )
%
% INPUT PARAMETERS: 
% - KinematicConstraints = [hmax vmax amax jmax ...] is the 
%   vector containing the kinematic constraints: hmax is the maximum 
%   displacement, vmax the maximum velocity, amax the maximum acceleration, 
%   jmax the maximum jerk, and so on. Note that with the smoothers based 
%   trajectory generator vmin = -vmax, amin = -amax, etc. The trajectory
%   always starts from zero initial position to the final postion hmax, beign hmax 
%   the required dispolacement. if a non-zero initial postion qi is desired,
%   it is sufficient to add this value to the filter output and considering
%   hmax = qf-qi, being qf the desired final position.
% - AngularFrequencies = [wr1 wr2 ...] is the vector of the RESONANT angular
%   frequencies (rad/s) affecting the plant, that the output trajectory won't 
%   excite (see the paper L. Biagiotti, C. Melchiorri, "FIR filters for 
%   online trajectory planning with time- and frequency-domain
%   specifications", Control Engineering Practice 20(12), December 2012)
% - SamplingTime is a scalar defining the sampling time of the discrete-time
%   trajectory generator. If not given, a continuous-time trajectory
%   generator is built (in this case max step size of the simulation must be 
%   small enough, in the simulation 1e-4s is assumed).
%
% OUTPUT: Simulink scheme in the continuous or discrete time domain for the
% simulation of the trajectory. Cut and paste the TrajectoryGenerator block
% (and the step input) in your simulink scheme. For complex trajectories
% use the block "Repeating Sequence Interpolated" with the option "Lookup 
% Method: Use Input Below" in place of the step input (hmax denotes the 
% maximum required displacement).
%
% EXAMPLES:
% BuildTrajectoryGenerator([1 10 3 0.4 0.4 4]) builds the continuous-time
% trajectory generator compliant (if the displacement is <= hmax = 1) with 
% vmax = max{q^(1)} = 10, amax = max{q^(2)} = 3, jmax = max{q^(3)} = 0.4, 
%  max{q^(4)} = 0.4, max{q^(5)} = 4.
% BuildTrajectoryGenerator([1 10 3 0.4 0.4 4], [], 0.01) defines the same 
% trajectory but in the discrete-time domain with sampling time 0.01 (because 
% of rounding operations the last bounded derivative may overcome the given limit 
% for a sampling period).
% BuildTrajectoryGenerator([0.03 0.1 1],[20 127]) builds the continuous-time
% trajectory generator compliant (if the displacement is <= hmax = 0.03) with 
% vmax = 0.1, amax = 1, and able to suppress vibrations at wr1 = 20 rad/s 
% and wr2 = 127 rad/s. The same trajectory generator can be defined in the
% discrete-time domain by adding the sampling time to the input parameters,
% e.g. BuildTrajectoryGenerator([0.03 0.1 1],[20 127], 0.0001)
%
% Copyright (c) 2019, Luigi Biagiotti (luigi.biagiotti@unimore.it)
function BuildTrajectoryGenerator( KinematicConstraints, AngularFrequencies, SamplingPeriod )

% Check of the input parameters
if ( ~exist('KinematicConstraints', 'var') )
    disp('At least the first parameter containing the maximum displacement is required');
    return
end

if (length(KinematicConstraints)>1)
    if (prod(double(KinematicConstraints(2:end)>0))<0.5)
        disp('All the kinematic constraints must be positive')
        return
    end  
   Tk = abs(KinematicConstraints(1:end-1)./KinematicConstraints(2:end)); % initial choice of the parameters
   Tparams = CheckConstraintsT(Tk); % Optimization of the kinematic parameters
else
   Tparams = [];
end

if ( exist('AngularFrequencies', 'var') )
    if (prod(double(AngularFrequencies>0))<0.5)
        disp('The frequencies must be positive')
        return
    end 
    Tw = 2*pi./AngularFrequencies;
    Tparams = MergeConstraintsT( Tparams, Tw ); % Merge kinematic and frequency constraints in an opmtial manner
end

if (length(Tparams)<1)
    disp('The trajectory generator must be composed by at least one smoother. Add more constraints!')
    return
end

if ( ~exist('SamplingPeriod', 'var') )% Continuous-time system
     BuildSimulinkScheme(KinematicConstraints,Tparams,1)
else % Continuous-time system
    if ( SamplingPeriod<=0 )
        disp('The sampling period must be a positive number. The continuous-time trajectory generator will be built.')
        BuildSimulinkScheme(KinematicConstraints,Tparams,1);
    elseif ( SamplingPeriod>=sum(Tparams)/10 )
        disp('The sampling period is too large (compare with the trajectory duration). The continuous-time trajectory generator will be built.')
        BuildSimulinkScheme(KinematicConstraints,Tparams,1);
    else
        BuildSimulinkScheme(KinematicConstraints,Tparams,0,SamplingPeriod) 
    end
end
%% Disable link to the library Smoothers.slx
blocks = find_system('TrajectoryGenerator', 'FollowLinks','on', 'BlockType', 'SubSystem');
for i=1:length(blocks)
     set_param(blocks{i},'LinkStatus','inactive');
end

end

function BuildSimulinkScheme(Constraints,Tparams,ContinuousTime,SamplingPeriod)
%% Automatic generation of Simulink scheme for trajectory planning
    sys = 'TrajectoryGenerator';
    bdclose(sys)
    new_system(sys) % Create the model
    open_system(sys) % Open the model

    n=length(Tparams); % order of the trajectory 
    % SamplingPeriod = 0.001;
    % ContinuosTime = 1;

    %% Continuous time generator
    if (ContinuousTime == 1) % Continuous-time scheme
        TotalDuration = sum(Tparams);
        x = 30;
        y = 80;
        w = 30;
        h = 30;
        pos = [x y-h/2 x+w y+h/2];
        add_block('built-in/step',[sys '/Step'],'Position',pos);
        dx = 2*w;
        dyout = 30;
        dy=60+(n-1)*dyout;
        w=150;
        pos = [pos(3)+dx y-dy/2  pos(3)+dx+w y+dy/2];
        add_block('simulink/Ports & Subsystems/Subsystem','TrajectoryGenerator/Trajectory Generator','Position',pos);
        Simulink.SubSystem.deleteContents('TrajectoryGenerator/Trajectory Generator')
        w = 100;
        pos = [pos(3)+dx y-dy/2  pos(3)+dx+w y+dy/2];
        add_block('simulink/Sinks/Scope','TrajectoryGenerator/Scope','Position',pos);
    % Annotation
        note = Simulink.Annotation('TrajectoryGenerator/Annotation');
        note.FixedWidth = 'on';
        note.position = [20, pos(4)+40 500 pos(4)+400];     
        NoteParameters = ['T1 = ' num2str(Tparams(1))];
        for i=2:n
            NoteParameters = [NoteParameters ', T' num2str(i) ' = ' num2str(Tparams(i))];
        end
        note.Text = ['Discrete-time trajectory generator of order ' num2str(n) ' composed by '  num2str(n) ' rectangular smoothers. The parameters of the smoothers are ' NoteParameters ' and the total duration of the rest-to-rest trajectory is ' num2str(TotalDuration) ' s.' ];
        
        x0 = 100;
        y0 = 60;
        w = 100;
        h = 60;
        hl(1) = 0;
        pos(1,:) = [x0 y0-h/2 x0+w y0+h/2];
        for i=1:n-1
            pos(i+1,:) = pos(i,:) + [1.5*w 0 1.5*w 0];
        end

        for j=1:n  %Blocks design
            for i=j:n
            BlockName{j,i} = ['TrajectoryGenerator/Trajectory Generator' strcat('/M',strcat(num2str(j),num2str(i)))];
            BlockNameShort{j,i} =  strcat('M',strcat(num2str(j),num2str(i)));
            add_block('Smoothers/M_s',BlockName{j,i},'Position',pos(i,:)+[0 hl(j) 0 hl(j)]);
            end
            hl(j+1) = hl(j)+h*1.4;
        end

        set_param('TrajectoryGenerator/Step','Time','0')
        set_param('TrajectoryGenerator/Step','After',num2str(Constraints(1)))
        set_param('TrajectoryGenerator/Step','SampleTime','0')

        for j=1:n  %Smoothers parameters
            for i=j:n
            set_param(BlockName{j,i},'T',num2str(Tparams(i)))    
            end
        end


        Pos = [20 y0-10 60 y0+10];
        add_block('simulink/Sources/In1',['TrajectoryGenerator/Trajectory Generator' '/x'],'Position',Pos);
        add_line('TrajectoryGenerator','Step/1','Trajectory Generator/1','autorouting','on')

        add_line('TrajectoryGenerator/Trajectory Generator','x/1','M11/1','autorouting','on')
        for i=1:n-1
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(1,i),'/1'),strcat(BlockNameShort(1,i+1),'/1'),'autorouting','on')
        end

        for j=1:n-1  %Connections design
            for i=j:n-1
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(j,i),'/2'),strcat(BlockNameShort(j+1,i+1),'/1'),'autorouting','on')
        end
        end

        % Add terminators to unconnected ports
        for j=2:n  %Blocks design
            for i=j:n
            add_block('simulink/Sinks/Terminator',['TrajectoryGenerator/Trajectory Generator' strcat('/Terminator',strcat(num2str(j),num2str(i)))],'Position',pos(i,:)+[110 hl(j)+h/4 15 hl(j)+h/4-(pos(i,4)-pos(i,2))*0.9]);
            set_param( ['TrajectoryGenerator/Trajectory Generator' strcat('/Terminator',strcat(num2str(j),num2str(i)))], 'ShowName' , 'off' ) % Hide names of terminators
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(j,i),'/1'), {strcat('Terminator',strcat(num2str(j),num2str(i)),'/1')},'autorouting','on')
            end
            hl(j+1) = hl(j)+h*1.4;
        end

        Pos = [pos(n,3)+50 y0-25 pos(n,3)+90   y0-5];
        add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' '/y'],'Position',Pos);

        Pos = [pos(n,3)+50 y0+5 pos(n,3)+90   y0+25];
        add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' '/y^(1)'],'Position',Pos);

        add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(1,n),'/1'),{'y/1'},'autorouting','on')
        
        for i=2:n
            Pos = Pos +[0 h 0 h]*1.4;
            add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' strcat(strcat('/y^(',num2str(i)),')')],'Position',Pos);
        end

        for i=1:n
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(i,n),'/2'),{strcat(strcat(strcat('y^(',num2str(i)),')'),'/1')},'autorouting','on')
        end

        %% Settings of Scope
        s = get_param('TrajectoryGenerator/Scope','ScopeConfiguration');
        s.NumInputPorts = num2str(n+1);
        s.LayoutDimensions = [(n+1) 1];
        s.DataLogging = true;
        s.DataLoggingVariableName = 'TrajectoryProfiles';
        s.DataLoggingSaveFormat = 'StructureWithTime';
        s.OpenAtSimulationStart = true;
        for i = 1:n+1
            add_line('TrajectoryGenerator',['Trajectory Generator/' num2str(i)],['Scope/' num2str(i)],'autorouting','on')
        end

        %% Simulation
        set_param('TrajectoryGenerator', 'MaxStep','1e-4','StopTime',num2str(TotalDuration));
        sim('TrajectoryGenerator')
    else
        %% Discrete-time scheme
        set_param(sys,'solver','FixedStepDiscrete','SolverType','Fixed-step','FixedStep',num2str(SamplingPeriod));
        Nparams = ceil(Tparams/SamplingPeriod);
        TotalDuration = sum(Nparams)*SamplingPeriod
        x = 30;
        y = 80;
        w = 30;
        h = 30;
        pos = [x y-h/2 x+w y+h/2];
        add_block('built-in/step',[sys '/Step'],'Position',pos);
        dx = 2*w;
        dyout = 30;
        dy=60+(n-1)*dyout;
        w=150;
        pos = [pos(3)+dx y-dy/2  pos(3)+dx+w y+dy/2];
        add_block('simulink/Ports & Subsystems/Subsystem','TrajectoryGenerator/Trajectory Generator','Position',pos);
        Simulink.SubSystem.deleteContents('TrajectoryGenerator/Trajectory Generator')

        w = 100;
        pos = [pos(3)+dx y-dy/2  pos(3)+dx+w y+dy/2];
        add_block('simulink/Sinks/Scope','TrajectoryGenerator/Scope','Position',pos);
        % Annotation
        note = Simulink.Annotation('TrajectoryGenerator/Annotation');
        note.FixedWidth = 'on';
        note.position = [20, pos(4)+40 500 pos(4)+400];
        
        NoteParameters = ['T1 = ' num2str(Tparams(1))];
        for i=2:n
            NoteParameters = [NoteParameters ', T' num2str(i) ' = ' num2str(Tparams(i))];
        end
        note.Text = ['Discrete-time trajectory generator of order ' num2str(n) ' composed by '  num2str(n) ' rectangular smoothers. The parameters of the smoothers are ' NoteParameters ' and the total duration of the rest-to-rest trajectory is ' num2str(TotalDuration) ' s.' ];
        
        x0 = 100;
        y0 = 60;
        w = 100;
        h = 60;
        hl(1) = 0;
        pos(1,:) = [x0 y0-h/2 x0+w y0+h/2];
        for i=1:n-1
            pos(i+1,:) = pos(i,:) + [1.5*w 0 1.5*w 0];
        end

        for j=1:n  %Blocks design
            for i=j:n
            BlockName{j,i} = ['TrajectoryGenerator/Trajectory Generator' strcat('/M',strcat(num2str(j),num2str(i)))];
            BlockNameShort{j,i} =  strcat('M',strcat(num2str(j),num2str(i)));
            add_block('Smoothers/M_z',BlockName{j,i},'Position',pos(i,:)+[0 hl(j) 0 hl(j)]);
            end
            hl(j+1) = hl(j)+h*1.4;
        end

        set_param('TrajectoryGenerator/Step','Time','0')
        set_param('TrajectoryGenerator/Step','After',num2str(Constraints(1)))
        set_param('TrajectoryGenerator/Step','SampleTime',num2str(SamplingPeriod))
        
        for j=1:n  %Smoothers parameters
            for i=j:n
            set_param(BlockName{j,i},'N',num2str(Nparams(i)))   
            set_param(BlockName{j,i},'Ts',num2str(SamplingPeriod)) 
            end
        end


        Pos = [20 y0-10 60 y0+10];
        add_block('simulink/Sources/In1',['TrajectoryGenerator/Trajectory Generator' '/x'],'Position',Pos);
        add_line('TrajectoryGenerator','Step/1','Trajectory Generator/1','autorouting','on')

        add_line('TrajectoryGenerator/Trajectory Generator','x/1','M11/1','autorouting','on')
        for i=1:n-1
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(1,i),'/1'),strcat(BlockNameShort(1,i+1),'/1'),'autorouting','on')
        end

        for j=1:n-1  %Connections design
            for i=j:n-1
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(j,i),'/2'),strcat(BlockNameShort(j+1,i+1),'/1'),'autorouting','on')
        end
        end

        % Add terminators to unconnected ports
        for j=2:n  %Blocks design
            for i=j:n
            add_block('simulink/Sinks/Terminator',['TrajectoryGenerator/Trajectory Generator' strcat('/Terminator',strcat(num2str(j),num2str(i)))],'Position',pos(i,:)+[110 hl(j)+h/4 15 hl(j)+h/4-(pos(i,4)-pos(i,2))*0.9]);
            set_param( ['TrajectoryGenerator/Trajectory Generator' strcat('/Terminator',strcat(num2str(j),num2str(i)))], 'ShowName' , 'off' ) % Hide names of terminators
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(j,i),'/1'), {strcat('Terminator',strcat(num2str(j),num2str(i)),'/1')},'autorouting','on')
            end
            hl(j+1) = hl(j)+h*1.4;
        end

        Pos = [pos(n,3)+50 y0-25 pos(n,3)+90   y0-5];
        add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' '/y'],'Position',Pos);

        Pos = [pos(n,3)+50 y0+5 pos(n,3)+90   y0+25];
        add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' '/y^(1)'],'Position',Pos);

        add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(1,n),'/1'),{'y/1'},'autorouting','on')

        for i=2:n
            Pos = Pos +[0 h 0 h]*1.4;
            add_block('simulink/Sinks/Out1',['TrajectoryGenerator/Trajectory Generator' strcat(strcat('/y^(',num2str(i)),')')],'Position',Pos);
        end

        for i=1:n
            add_line('TrajectoryGenerator/Trajectory Generator',strcat(BlockNameShort(i,n),'/2'),{strcat(strcat(strcat('y^(',num2str(i)),')'),'/1')},'autorouting','on')
        end

        %% Settings of Scope
        s = get_param('TrajectoryGenerator/Scope','ScopeConfiguration');
        s.NumInputPorts = num2str(n+1);
        s.LayoutDimensions = [(n+1) 1];
        s.DataLogging = true;
        s.DataLoggingVariableName = 'TrajectoryProfiles';
        s.DataLoggingSaveFormat = 'StructureWithTime';
        s.OpenAtSimulationStart = true;
        for i = 1:n+1
            add_line('TrajectoryGenerator',['Trajectory Generator/' num2str(i)],['Scope/' num2str(i)],'autorouting','on')
        end

        %% Simulation
        set_param('TrajectoryGenerator','StopTime',num2str(TotalDuration));
        sim('TrajectoryGenerator') 
    end
end

% function [ Tout ] = CheckConstraintsT(Tin)
%     amax = 0.999999;
%     amin = 0.95;
%     n = length(Tin);
%     Tout=Tin;
%     for i=n-1:-1:1
%           if Tout(i)< sum(Tout(i+1:end))
%             a = -sum(Tout(i+2:end))/2/Tout(i+1)+sqrt((sum(Tout(i+2:end))/2/Tout(i+1))^2+Tout(i)/Tout(i+1));
%             a = min([max([a amin]), amax]);
%             Tout(i) = Tout(i)/a;
%             Tout(i+1) = Tout(i+1)*a;
%             Tout  = CheckConstraintsT(Tout);
%          end
%     end
% end

function [ Tout ] = CheckConstraintsT( Tin )
amax = 0.999999;
amin = 0.95;
n = length(Tin);
Tout=Tin;
if ( Tout(n-1) < Tout(n) )
    Tout(n) = sqrt(Tout(n-1)*Tout(n));
    Tout(n-1) = Tout(n);
end
for i=n-2:-1:1
          if ( Tout(i)< Tout(i+1) + Tout(i+2) ) 
            a = -Tout(i+2)/2/Tout(i+1)+sqrt((Tout(i+2)/2/Tout(i+1))^2+Tout(i)/Tout(i+1));   
            a = min([max([a amin]), amax]);
            Tout(i) = Tout(i)/a;
            Tout(i+1) = Tout(i+1)*a;
            Tout  = CheckConstraintsT( Tout );
          end
end
end

function [Tstar] = MergeConstraintsT( T, Tw )
    n = length(T);
    m = length(Tw);
    j = 1;
    Tstar = [];
    for i = 1: n
            if (j <= m) & (Tw(j) >= T(i))
                Tstar(i) = Tw(j);
                j = j + 1;
            else
                Tstar(i) = T(i);
            end
    end
    Tstar = [Tstar Tw(j:end)]; 
end