class MissionPlanner
{
    constructor() {
        this.addHTML();
        this.addDrawTypes();
        //this.addCallbacks();
        
    }

    addHTML() {

        this.htmlId = 'sideBar-left-missionPlanner-content';

        let missionPlannerHtmlList = [];

        // Mission Dropdown list
        let missionList = ['New mission'];
        let missionListTotal = missionList.concat(M.MISSION_LIST.getMissionList());
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, missionListTotal, 'New Mission'));

        // UAV Dropdown list
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-UAVList`, M.UAV_LIST.getUavList(), M.UAV_LIST.getUavList()[0]));

        // Heigh input
        let heightInput = HTMLUtils.addDict('input', `${this.htmlId}-heightInput`, {'class': 'form-control', 'required': 'required',}, 'text', '1');
        let heightBtn   = HTMLUtils.addDict('button', `${this.htmlId}-heighBtn`, {'class': 'btn btn-primary'}, 'Set Height (m)');
        let heightRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [heightInput, heightBtn], {'class': 'col-md-6'});
        missionPlannerHtmlList.push(heightRow);


        // Buttons for change draw mode
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-mouse`,  {'class': 'btn btn-primary m-1',}, `<i class="fas fa-mouse-pointer"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-edit`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-edit"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-delete`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-eraser"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-move`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-arrows-alt"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-rotate`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-sync-alt"></i>`));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-PoI`,     {'class': 'btn btn-primary',}, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-WP`,      {'class': 'btn btn-primary',}, `WayPoint  <i class="fa-solid fa-location-pin"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-path`,    {'class': 'btn btn-primary',}, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-area`,    {'class': 'btn btn-primary',}, `Area <i class="fas fa-draw-polygon"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-cArea`,   {'class': 'btn btn-primary',}, `Circular area <i class="far fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-land`,    {'class': 'btn btn-primary',}, `Land point <i class="fas fa-h-square"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-remove`,  {'class': 'btn btn-warning'},  'Remove all draw'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-confirm`, {'class': 'btn btn-success'},  'Confirm mission'));
        missionPlannerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Global collapse
        let missionPlannerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-addToMapCollapse`, {}, 'Planner', true, missionPlannerHtmlList);

        // TODO: Save Mission Planner Dict on a json file
        
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionPlannerCollapse]);
    }

    addDrawTypes() {
        this.layer = new Layer('user', 'New Mission', M.UAV_LIST.getUavList()[0], 1);
        this.pointOfInterest = new PointOfInterest();
        this.wayPoint = new WayPoint();
        this.path = new Path();
        this.area = new Area();
        this.carea = new CircularArea();
        this.landPoint = new LandPoint()
    }

    addCallbacks() {
        
        // Mission Dropdown list
        Utils.addButtonsCallback(`${this.htmlId }-MissionList-item`, this.missionListCallback, [this.layer]);

        // UAV Dropdown list
        Utils.addButtonsCallback(`${this.htmlId }-UAVList-item`, this.uavListCallback, [this.layer]);

        // Heigh input
        Utils.addFormCallback(`${this.htmlId}-heighBtn`, [`${this.htmlId}-heightInput`], ['height'], this.heightCallback, [this.layer]);

        // Buttons for change draw mode
        Utils.addButtonCallback(`${this.htmlId}-mouse`,   DrawController.drawMouse, []);
        Utils.addButtonCallback(`${this.htmlId}-edit`,    DrawController.drawEdit, []);
        Utils.addButtonCallback(`${this.htmlId}-delete`,  DrawController.drawDelete, []);
        Utils.addButtonCallback(`${this.htmlId}-move`,    DrawController.drawMove, []);
        Utils.addButtonCallback(`${this.htmlId}-rotate`,  DrawController.drawRotate, []);
        
        // Buttons for draw mission
        Utils.addButtonCallback(`${this.htmlId}-PoI`,   this.userDrawCallbacks, [this.pointOfInterest, this.layer]);
        Utils.addButtonCallback(`${this.htmlId}-WP`,    this.userDrawCallbacks, [this.wayPoint,        this.layer]);
        Utils.addButtonCallback(`${this.htmlId}-path`,  this.userDrawCallbacks, [this.path,            this.layer]);
        Utils.addButtonCallback(`${this.htmlId}-area`,  this.userDrawCallbacks, [this.area,            this.layer]);
        Utils.addButtonCallback(`${this.htmlId}-cArea`, this.userDrawCallbacks, [this.carea,           this.layer]);
        Utils.addButtonCallback(`${this.htmlId}-land`,  this.userDrawCallbacks, [this.landPoint,       this.layer]);

        Utils.addButtonCallback(`${this.htmlId}-remove`,  DrawController.drawRemoveAll, []);
        Utils.addButtonCallback(`${this.htmlId}-confirm`, this.confirmCallback, []);

        // add listener to reset draw when press ESC key
        document.addEventListener('keydown', function(e){
            switch (e.key) {
                case 'Escape':
                    console.log('ESC pressed');
                    DrawController.drawMouse();
                    break;
                default:
                    break;
            }
        });
    }

    missionListCallback(e, args) {
        let button = document.getElementById('sideBar-left-missionPlanner-content-MissionList-DropDown-Btn');
        button.innerHTML = e.innerHTML;
        args[0].updateMissionId(e.innerHTML);
    }

    uavListCallback(e, args) {
        args[0].updateUavList(e.innerHTML);
    }

    heightCallback(arg, input) {
        arg[0].updateHeight(input['height']);
    }

    userDrawCallbacks(args=[]) {
        args[0].userDraw(args[1]);
    }

    confirmCallback(args=[]) {
        // TODO: websocket -> send mission to server
        console.log('confirm mission');
    }   
}
