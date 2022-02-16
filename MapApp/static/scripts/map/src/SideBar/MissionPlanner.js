class MissionPlanner
{
    constructor() {
        this.htmlId = 'sideBar-left-missionPlanner-content';

        this.initialized = false;
        M.UAV_MANAGER.addUavListCallback(this.updateUavListCallback.bind(this), this);
        M.MISSION_MANAGER.addMissionListCallback(this.updateMissionListCallback.bind(this), this);

        this.selectedMission = null;
        this.selectedUavs = [];
        this.selectedHeight = [];

        this.addPlannerHTML();
        this.addDrawTypes();
    }

    _checkInitalize() {
        if (!this.initialized) {
            this.addConfirmHTML();
            this.initialized = true;
        }
    }

    // #region Planner
    addPlannerHTML() {
        let mPlannerList = [];

        // Heigh input
        let heightInput = HTMLUtils.addDict('input',  `${this.htmlId}-heightInput`, {'class': 'form-control', 'required': 'required',}, 'text', '1');
        let heightBtn   = HTMLUtils.addDict('button', `${this.htmlId}-heighBtn`, {'class': 'btn btn-primary'}, 'Set Height (m)');
        let heightRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [heightInput, heightBtn], {'class': 'col-md-6'});
        mPlannerList.push(heightRow);

        let heightInputMin  = HTMLUtils.addDict('input', `${this.htmlId}-heightInputMin`, {'class': 'form-control', 'required': 'required',}, 'text', 'Min');
        let heightInputMax  = HTMLUtils.addDict('input', `${this.htmlId}-heightInputMax`, {'class': 'form-control', 'required': 'required',}, 'text', 'Max');
        let heightRangeBtn  = HTMLUtils.addDict('button', `${this.htmlId}-heighRangeBtn`, {'class': 'btn btn-primary'}, 'Set Height (m)');
        let heightRangeRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [heightInputMin, heightInputMax, heightRangeBtn], {'class': 'col heightRange'});
        mPlannerList.push(heightRangeRow);


        // Buttons for change draw mode
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-mouse`,  {'class': 'btn btn-primary m-1',}, `<i class="fas fa-mouse-pointer"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-edit`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-edit"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-delete`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-eraser"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-move`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-arrows-alt"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-rotate`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-sync-alt"></i>`));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-takeOff`, {'class': 'btn btn-primary',}, `Take off <i class="fa-solid fa-t"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-PoI`,     {'class': 'btn btn-primary',}, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-WP`,      {'class': 'btn btn-primary',}, `WayPoint  <i class="fa-solid fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-path`,    {'class': 'btn btn-primary',}, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-area`,    {'class': 'btn btn-primary',}, `Area <i class="fas fa-draw-polygon"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-cArea`,   {'class': 'btn btn-primary',}, `Circular area <i class="far fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-land`,    {'class': 'btn btn-primary',}, `Land point <i class="fas fa-h-square"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-remove`,  {'class': 'btn btn-warning'},  'Remove all draw'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-save`,    {'class': 'btn btn-success'},  'Save Mission in file'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-load`,    {'class': 'btn btn-success'},  'Load Mission from file'));
        mPlannerList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Planner collapse
        let missionPlannerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-PlannerCollapse`, {}, 'Planner', true, mPlannerList);
        
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionPlannerCollapse]);

        // Makes height range button looks pretty
        let btnRange = document.getElementsByClassName('heightRange');
        btnRange[2].setAttribute('class', 'col-6');

        this.addPlannerCallbacks();
    }

    addPlannerCallbacks() {

        // Heigh input
        Utils.addFormCallback(`${this.htmlId}-heighBtn`, [`${this.htmlId}-heightInput`], ['height'], this.heightCallback.bind(this));
        Utils.addFormCallback(`${this.htmlId}-heighRangeBtn`, [`${this.htmlId}-heightInputMin`, `${this.htmlId}-heightInputMax`], ['heightMin', 'heightMax'], this.heightRangeCallback.bind(this));

        // Buttons for change draw mode
        Utils.addButtonCallback(`${this.htmlId}-mouse`,   DrawController.drawMouse, []);
        Utils.addButtonCallback(`${this.htmlId}-edit`,    DrawController.drawEdit, []);
        Utils.addButtonCallback(`${this.htmlId}-delete`,  DrawController.drawDelete, []);
        Utils.addButtonCallback(`${this.htmlId}-move`,    DrawController.drawMove, []);
        Utils.addButtonCallback(`${this.htmlId}-rotate`,  DrawController.drawRotate, []);
        
        // Buttons for draw mission
        Utils.addButtonCallback(`${this.htmlId}-takeOff`,   this.userDrawCallbacks.bind(this), [this.takeOffPont]);
        Utils.addButtonCallback(`${this.htmlId}-PoI`,   this.userDrawCallbacks.bind(this), [this.pointOfInterest]);
        Utils.addButtonCallback(`${this.htmlId}-WP`,    this.userDrawCallbacks.bind(this), [this.wayPoint]);
        Utils.addButtonCallback(`${this.htmlId}-path`,  this.userDrawCallbacks.bind(this), [this.path]);
        Utils.addButtonCallback(`${this.htmlId}-area`,  this.userDrawCallbacks.bind(this), [this.area]);
        Utils.addButtonCallback(`${this.htmlId}-cArea`, this.userDrawCallbacks.bind(this), [this.carea]);
        Utils.addButtonCallback(`${this.htmlId}-land`,  this.userDrawCallbacks.bind(this), [this.landPoint]);

        Utils.addButtonCallback(`${this.htmlId}-remove`,  DrawController.drawRemoveAll, []);

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

    addDrawTypes() {
        let fillColor = '#b3b3b3';
        let borderColor = '#7f7f7f';

        let userDrawOptions = {'color': M.drawColor};
        this.pointOfInterest = new PointOfInterest(fillColor, borderColor);
        this.wayPoint = new WayPoint(fillColor, borderColor);
        this.path = new Path({}, userDrawOptions);
        this.area = new Area({}, userDrawOptions);
        this.carea = new CircularArea({}, userDrawOptions);
        this.landPoint = new LandPoint(fillColor, borderColor);
        this.takeOffPont = new TakeOffPoint(fillColor, borderColor);
    }

    // #region Callbacks
    heightCallback(arg, input) {
        this.selectedHeight = [input['height'], input['height']];
    }

    heightRangeCallback(arg, input) {
        this.selectedHeight = [input['heightMin'], input['heightMax']];
    }

    userDrawCallbacks(args=[]) {
        args[0].userDraw({'height': this.selectedHeight});
    }

    confirmCallback(args=[]) {
        // TODO: websocket -> send mission to server
        console.log('confirm mission');
        let layers = M.getLayers();

        console.log(layers);

        /*
        let drawLayers = [];
        for (let i=0; i<layers.length; i++) {
            let options = layers[i].pm.options;
            if (options.status == 'draw' && options.missionId == this.layer.getMissionId()) {

                let layer_info = {
                    'name': options.name,
                    'height': options.height,
                };

                // add layer info to a list
                switch(options.type) {
                    case 'Marker':
                        layer_info['layer'] = layers[i]._latlng;
                        break;
                    case 'Circle':
                    case 'CircleMarker':
                        layer_info['layer'] = [layers[i]._latlng, layers[i]._mRadius];
                        break;
                    case 'Line':
                    case 'Polygon':
                    case 'Rectangle':
                        layer_info['layer'] = layers[i]._latlngs;
                        break;
                }
                drawLayers.push(layer_info)
            }   
        }

        if (drawLayers.length > 0) {
            M.WS.requestConfirmMission(
                this.selectedMission,
                this.selectedUavs,
                drawLayers
            );      
        }
        */
    }
    // #endregion

    // #endregion

    // #region Confirm
    addConfirmHTML() {
        let mConfirmList = [];

        // Mission Dropdown list
        let missionList = ['New mission'];
        let missionListTotal = missionList.concat(M.MISSION_MANAGER.getMissionList());
        mConfirmList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, missionListTotal, 'New Mission'));

        // UAV picker
        let uavPickerList = HTMLUtils.addDict('checkBoxes', `${this.htmlId}-UAVPicker`, {}, M.UAV_MANAGER.getUavList());
        mConfirmList.push(HTMLUtils.addDict('collapse', `${this.htmlId}-UAVCollapse`, {}, 'UAV Picker', true, [uavPickerList]));


        // Buttons for confirm mission
        let splitBtnConfirm = [];
        splitBtnConfirm.push(HTMLUtils.addDict('button', `${this.htmlId}-SaveMission`, {'class': 'btn btn-primary'},  'Save Mission'));
        splitBtnConfirm.push(HTMLUtils.addDict('button', `${this.htmlId}-confirm`, {'class': 'btn btn-success'},  'Confirm mission'));
        mConfirmList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtnConfirm, {'class': 'row m-1'}));
        
        // Confirm collapse
        let missionConfirmCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-ConfirmCollapse`, {}, 'Confirm', false, mConfirmList);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionConfirmCollapse]);
    }

    addConfirmCallbacks() {
        Utils.addButtonCallback(`${this.htmlId}-confirm`, this.confirmCallback, []);
    }

    // #region Callbacks
    addBntDropDownMissionCallback() {
        Utils.addButtonsCallback(`${this.htmlId }-MissionList-item`, this.clickMissionListCallback.bind(this));
    }

    updateUavListCallback(myargs, args) {
        this._checkInitalize();
        HTMLUtils.updateCheckBoxes(`${this.htmlId}-UAVPicker`, M.UAV_MANAGER.getUavList());
    }

    updateMissionListCallback(myargs, args) {
        if (this.initialized) {
            HTMLUtils.updateDropDown(`${this.htmlId}-MissionList`, ['New Mission'].concat(M.MISSION_MANAGER.getMissionList()));
            this.addBntDropDownMissionCallback();
        }
    }

    clickMissionListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-MissionList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.selectedMission = e.innerHTML;
    }

    clickUavListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-UAVList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.selectedUavs = e.innerHTML;
    }
    // #endregion
    // #endregion
}
