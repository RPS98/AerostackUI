class MissionPlanner
{
    constructor() {
        this.htmlId = 'sideBar-left-missionPlanner-content';

        this.initialized = false;
        M.addUavListCallback(this.updateUavListCallback.bind(this), this);
        M.addMissionListCallback(this.updateMissionListCallback.bind(this), this);
    }

    addHTML() {
        let missionPlannerHtmlList = [];

        // Mission Dropdown list
        let missionList = ['New mission'];
        let missionListTotal = missionList.concat(M.getMissionList());
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, missionListTotal, 'New Mission'));

        // UAV Dropdown list
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-UAVList`, M.getUavList(), M.getUavList()[0]));

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

        this.addDrawTypes();
    }

    addDrawTypes() {
        this.layer = new Layer('user', 'New Mission', M.getUavList()[0], 1);
        this.pointOfInterest = new PointOfInterest();
        this.wayPoint = new WayPoint();
        this.path = new Path();
        this.area = new Area();
        this.carea = new CircularArea();
        this.landPoint = new LandPoint()

        this.addCallbacks();
    }

    addCallbacks() {

        // Heigh input
        Utils.addFormCallback(`${this.htmlId}-heighBtn`, [`${this.htmlId}-heightInput`], ['height'], this.heightCallback.bind(this));

        // Buttons for change draw mode
        Utils.addButtonCallback(`${this.htmlId}-mouse`,   DrawController.drawMouse, []);
        Utils.addButtonCallback(`${this.htmlId}-edit`,    DrawController.drawEdit, []);
        Utils.addButtonCallback(`${this.htmlId}-delete`,  DrawController.drawDelete, []);
        Utils.addButtonCallback(`${this.htmlId}-move`,    DrawController.drawMove, []);
        Utils.addButtonCallback(`${this.htmlId}-rotate`,  DrawController.drawRotate, []);
        
        // Buttons for draw mission
        Utils.addButtonCallback(`${this.htmlId}-PoI`,   this.userDrawCallbacks.bind(this), [this.pointOfInterest]);
        Utils.addButtonCallback(`${this.htmlId}-WP`,    this.userDrawCallbacks.bind(this), [this.wayPoint]);
        Utils.addButtonCallback(`${this.htmlId}-path`,  this.userDrawCallbacks.bind(this), [this.path]);
        Utils.addButtonCallback(`${this.htmlId}-area`,  this.userDrawCallbacks.bind(this), [this.area]);
        Utils.addButtonCallback(`${this.htmlId}-cArea`, this.userDrawCallbacks.bind(this), [this.carea]);
        Utils.addButtonCallback(`${this.htmlId}-land`,  this.userDrawCallbacks.bind(this), [this.landPoint]);

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

    addDropDownUavCallback() {
        Utils.addButtonsCallback(`${this.htmlId }-UAVList-item`, this.clickUavListCallback.bind(this));
    }

    addDropDownMissionCallback() {
        Utils.addButtonsCallback(`${this.htmlId }-MissionList-item`, this.clickMissionListCallback.bind(this));
    }

    _checkInitalize() {
        if (!this.initialized) {
            this.addHTML();
            this.initialized = true;
        }
    }

    updateUavListCallback(myargs, args) {
        this._checkInitalize();
        HTMLUtils.updateDropDown(`${this.htmlId}-UAVList`, M.getUavList());
        this.addDropDownUavCallback();
    }

    updateMissionListCallback(myargs, args) {
        this._checkInitalize();
        HTMLUtils.updateDropDown(`${this.htmlId}-MissionList`, ['New Mission'].concat(M.getMissionList()));
        this.addDropDownMissionCallback();
    }

    clickMissionListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-MissionList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.layer.updateMissionId(e.innerHTML);
    }

    clickUavListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-UAVList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.layer.updateUavList(e.innerHTML);
    }

    heightCallback(arg, input) {
        this.layer.updateHeight(input['height']);
    }

    userDrawCallbacks(args=[]) {
        args[0].userDraw(this.layer);
    }

    confirmCallback(args=[]) {
        // TODO: websocket -> send mission to server
        console.log('confirm mission');
    }   
}
