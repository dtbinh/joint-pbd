function point = get_end_position(fig,varargin)

% option to delete/not delete data aftere finshed demonstration
delete_trace = 1;
if (nargin>1)
    delete_trace = varargin{1};
end
    

% to store the data
X = [];
% flag for signaling that the demonstration has ended
finished = 0;

% select our figure as gcf
figure(fig);
hold on
% disable any figure modes
zoom off
rotate3d off
pan off
brush off
datacursormode off

set(fig,'WindowButtonDownFcn',@(h,e)record_current_point(h,e));
set(fig,'WindowButtonUpFcn',[]);
set(fig,'Pointer','circle');

hp = gobjects(0);

% wait until demonstration is finished
while(~finished)
    pause(0.1);
end
% set the return value
point = x;
set(fig,'Pointer','arrow');
return

    function ret = record_current_point(h,e)
        if(strcmp(get(gcf,'SelectionType'),'normal'))
            x = get(gca,'Currentpoint');
            x = x(1,1:2)';
            x = [x ;0];
            hp = [hp, plot(x(1),x(2),'r.','markersize',20)];
            disp(sprintf('Point chosen: %s %s', x(1),x(2)));
            set(gcf,'WindowButtonUpFcn',[]);
            set(gcf,'WindowButtonDownFcn',[]);
            if(delete_trace)
                delete(hp);
            end
            finished = 1;
        end
    end
end