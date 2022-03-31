class MissionPlanner {
    constructor() {
        this.htmlId = 'sideBar-left-missionPlanner-content';

        this.initialized = false;
        M.UAV_MANAGER.addInfoAddCallback(this.updateUavListCallback.bind(this));
        M.MISSION_MANAGER.addInfoAddCallback(this.updateMissionListCallback.bind(this));

        this.selectedMission = 'New Mission';
        this.selectedUavs = {};
        this.selectedHeight = [config.SideBars.MissionPlanner.defaultHeight, config.SideBars.MissionPlanner.defaultHeight];
        this.selectedSpeed = config.SideBars.MissionPlanner.defaultSpeed;

        this.addDrawTypes();

        this.addPlannerHTML();
    }

    _checkInitalize() {
        if (!this.initialized) {
            this.addConfirmHTML();
            this.initialized = true;
        }
    }

    // #region Planner
    addDrawTypes() {
        let status = 'draw';

        let fillColor = '#b3b3b3';
        let borderColor = '#7f7f7f';
        let layerOptions = {
            'continueDrawing': false,
            'fillColor': fillColor,
            'borderColor': borderColor,
        }

        this.pointOfInterest = new PointOfInterest(status, undefined, layerOptions);
        // this.wayPoint = new WayPoint(status, undefined, layerOptions);
        this.wayPoint = new WayPoint(status, undefined, Object.assign({}, layerOptions, { 'continueDrawing': true })); // TODO: not supported
        this.landPoint = new LandPoint(status, undefined, layerOptions);
        this.takeOffPoint = new TakeOffPoint(status, undefined, layerOptions);

        let drawDefaultColor = '#B3B3B3';
        let layerOptions2 = {
            'continueDrawing': false,
            'color': drawDefaultColor,
        }

        this.path = new Path(status, undefined, layerOptions2);
        this.area = new Area(status, undefined, layerOptions2);
        this.carea = new CircularArea(status, undefined, layerOptions2);
    }

    addPlannerHTML() {
        let mPlannerList = [];

        // Speed input
        let speedInput = HTMLUtils.addDict('input', `${this.htmlId}-speedInput`, { 'class': 'form-control', 'required': 'required', }, 'text', `${this.selectedSpeed}`);
        let speedBtn = HTMLUtils.addDict('button', `${this.htmlId}-speedBtn`, { 'class': 'btn btn-primary' }, 'Set Speed (m/s)');
        let speedRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [speedInput, speedBtn], { 'class': 'col-md-6' });
        mPlannerList.push(speedRow);


        // Heigh input
        let heightInput = HTMLUtils.addDict('input', `${this.htmlId}-heightInput`, { 'class': 'form-control', 'required': 'required', }, 'text', `${this.selectedHeight[1]}`);
        let heightBtn = HTMLUtils.addDict('button', `${this.htmlId}-heightBtn`, { 'class': 'btn btn-primary' }, 'Set Height (m)');
        let heightRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [heightInput, heightBtn], { 'class': 'col-md-6' });
        mPlannerList.push(heightRow);

        let heightInputMin = HTMLUtils.addDict('input', `${this.htmlId}-heightInputMin`, { 'class': 'form-control', 'required': 'required', }, 'text', 'Min');
        let heightInputMax = HTMLUtils.addDict('input', `${this.htmlId}-heightInputMax`, { 'class': 'form-control', 'required': 'required', }, 'text', 'Max');
        let heightRangeBtn = HTMLUtils.addDict('button', `${this.htmlId}-heighRangeBtn`, { 'class': 'btn btn-primary' }, 'Set Height (m)');

        let heightInputMinDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMin]);
        let heightInputMaxDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [heightInputMax]);
        let heightRangeBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [heightRangeBtn]);

        let heightRangeRow = HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [heightInputMinDiv, heightInputMaxDiv, heightRangeBtnDiv]);
        mPlannerList.push(heightRangeRow);

        // Buttons for change draw mode
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-mouse`, { 'class': 'btn btn-primary m-1', }, `<i class="fas fa-mouse-pointer"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-edit`, { 'class': 'btn btn-primary m-1', }, `<i class="fas fa-edit"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-delete`, { 'class': 'btn btn-primary m-1', }, `<i class="fas fa-eraser"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-move`, { 'class': 'btn btn-primary m-1', }, `<i class="fas fa-arrows-alt"></i>`));
        mPlannerList.push(HTMLUtils.addDict('button', `${this.htmlId}-rotate`, { 'class': 'btn btn-primary m-1', }, `<i class="fas fa-sync-alt"></i>`));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-takeOff`, { 'class': 'btn btn-primary', }, `Take off <i class="fa-solid fa-t"></i>`));
        // splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-PoI`, { 'class': 'btn btn-primary', }, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-WP`, { 'class': 'btn btn-primary', }, `WayPoint  <i class="fa-solid fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-path`, { 'class': 'btn btn-primary', }, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-area`, { 'class': 'btn btn-primary', }, `Area <i class="fas fa-draw-polygon"></i>`));
        // splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-cArea`, { 'class': 'btn btn-primary', }, `Circular area <i class="far fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-land`, { 'class': 'btn btn-primary', }, `Land point <i class="fas fa-h-square"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-remove`, { 'class': 'btn btn-warning' }, 'Remove all draw'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-save`, { 'class': 'btn btn-success' }, 'Save Mission in file'));

        // splitBtn.push(HTMLUtils.addToExistingElement(`${this.htmlId}`, [HTMLUtils.addDict('fileInput', `${this.htmlId}-missionFile`, {}, 'Choose Mission File', '')]));

        mPlannerList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, { 'class': 'row m-1' }));

        mPlannerList.push(HTMLUtils.addDict('fileInput', `${this.htmlId}-missionFile`, {}, 'Choose Mission File', ''));

        // Planner collapse
        let missionPlannerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-PlannerCollapse`, {}, 'Planner', true, mPlannerList);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionPlannerCollapse]);

        this.addPlannerCallbacks();
    }

    addPlannerCallbacks() {

        // Heigh input
        Utils.addFormCallback(`${this.htmlId}-heightBtn`, [`${this.htmlId}-heightInput`], ['height'], this.heightCallback.bind(this));
        Utils.addFormCallback(`${this.htmlId}-heighRangeBtn`, [`${this.htmlId}-heightInputMin`, `${this.htmlId}-heightInputMax`], ['heightMin', 'heightMax'], this.heightRangeCallback.bind(this));

        // Buttons for change draw mode
        Utils.addButtonCallback(`${this.htmlId}-mouse`, DrawController.drawMouse, []);
        Utils.addButtonCallback(`${this.htmlId}-edit`, DrawController.drawEdit, []);
        Utils.addButtonCallback(`${this.htmlId}-delete`, DrawController.drawDelete, []);
        Utils.addButtonCallback(`${this.htmlId}-move`, DrawController.drawMove, []);
        Utils.addButtonCallback(`${this.htmlId}-rotate`, DrawController.drawRotate, []);

        // Buttons for draw mission
        Utils.addButtonCallback(`${this.htmlId}-takeOff`, this.userDrawCallbacks.bind(this), [this.takeOffPoint]);
        // Utils.addButtonCallback(`${this.htmlId}-PoI`, this.userDrawCallbacks.bind(this), [this.pointOfInterest]);
        Utils.addButtonCallback(`${this.htmlId}-WP`, this.userDrawCallbacks.bind(this), [this.wayPoint]);
        Utils.addButtonCallback(`${this.htmlId}-path`, this.userDrawCallbacks.bind(this), [this.path]);
        Utils.addButtonCallback(`${this.htmlId}-area`, this.userDrawCallbacks.bind(this), [this.area]);
        // Utils.addButtonCallback(`${this.htmlId}-cArea`, this.userDrawCallbacks.bind(this), [this.carea]);
        Utils.addButtonCallback(`${this.htmlId}-land`, this.userDrawCallbacks.bind(this), [this.landPoint]);

        Utils.addButtonCallback(`${this.htmlId}-remove`, DrawController.drawRemoveAll, []);
        Utils.addButtonCallback(`${this.htmlId}-save`, this.saveMissionCallback.bind(this));
        // Utils.addButtonCallback(`${this.htmlId}-load`, this.loadMission.bind(this));

        Utils.addFileCallback(`${this.htmlId}-missionFile`, this.loadMissionCallback.bind(this));

        // add listener to reset draw when press ESC key
        document.addEventListener('keydown', function (e) {
            switch (e.key) {
                case 'Escape':
                    //console.log('ESC pressed');
                    DrawController.drawMouse();
                    break;
                default:
                    break;
            }
        });
    }

    // #region Callbacks
    heightCallback(arg, input) {
        this.selectedHeight = [input['height'], input['height']];
    }

    heightRangeCallback(arg, input) {
        this.selectedHeight = [input['heightMin'], input['heightMax']];
    }

    userDrawCallbacks(args = []) {
        args[0][0].userDraw({ 'height': this.selectedHeight });
    }

    // #endregion

    // #endregion

    // #region Confirm
    addConfirmHTML() {
        let mConfirmList = [];

        // Mission Dropdown list
        let missionList = ['New mission'];
        let missionListTotal = missionList.concat(M.MISSION_MANAGER.getList());
        mConfirmList.push(HTMLUtils.initDropDown(`${this.htmlId}-MissionList`, missionListTotal, 'New Mission'));

        // UAV picker
        // let list = [['none', false], ['auto', true]];
        let list = [['none', false]];
        let uavPickerList = M.getUavPickerDict('checkbox', `${this.htmlId}-UAVPicker`, list, this.clickUavListCallback.bind(this));

        mConfirmList.push(HTMLUtils.addDict('collapse', `${this.htmlId}-UAVCollapse`, {}, 'UAV Picker', true, [uavPickerList]));

        // Buttons for confirm mission
        let splitBtnConfirm = [];
        splitBtnConfirm.push(HTMLUtils.addDict('button', `${this.htmlId}-confirm`, { 'class': 'btn btn-success', disabled: true }, 'Confirm mission'));
        mConfirmList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtnConfirm, { 'class': 'row m-1' }));

        // Confirm collapse
        let missionConfirmCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-ConfirmCollapse`, {}, 'Confirm', false, mConfirmList);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionConfirmCollapse]);

        this.addConfirmCallbacks();
    }

    addConfirmCallbacks() {
        Utils.addButtonCallback(`${this.htmlId}-confirm`, this.confirmBtnCallback.bind(this), []);
        M.uavPickerInitiliazeCallback(`${this.htmlId}-UAVPicker`);
    }


    missionInterpreterTakeOffPoint(layerInfo, missionLayer, selectedUavListTakeOff, validation, info) {
        let minDistance = Math.pow(10, -6);

        let takeOffPosition = layerInfo.layer.getLatLng();
        for (let j = 0; j < selectedUavListTakeOff.length; j++) {
            let pose = M.UAV_MANAGER.getDictById(selectedUavListTakeOff[j]).pose;
            let distance = Utils.distance(
                takeOffPosition.lat,
                takeOffPosition.lng,
                pose.lat,
                pose.lng
            );

            if (distance < minDistance) {

                missionLayer['uavList'].push(selectedUavListTakeOff[j]);
                selectedUavListTakeOff.splice(j, 1);
                return;
            }
        }

        if (missionLayer['uavList'].length == 0) {
            validation = false;
            info.push(`Take off point is not connected to any UAV`);
        }
    }

    missionInterpreterMarkers(layerInfo, missionLayer, selectedUavList, validation, info) {

        let uavList = Object.keys(layerInfo.drawManager.options.uavList);

        if (uavList.length > 1) {
            validation = false;
            info.push(`More than one UAV is selected for layer ${layerInfo.layer.options.name}`);
            return;
        }

        let selection = String(uavList[0]);

        if (selection == 'auto') {
            if (selectedUavList.length == 1) {
                missionLayer['uavList'].push(selectedUavList[0]);
            } else {
                validation = false;
                info.push(`${layerInfo.drawManager.options.name} layer has error with UAV selected. Auto option is not allowed with multiple UAVs`);
            }
        } else {
            if (selectedUavList.includes(selection)) {
                missionLayer['uavList'].push(selection);
            } else {
                validation = false;
                info.push(`${layerInfo.drawManager.options.name} layer has error with UAV selected. UAV associated to this layer is not selected`);
            }
        }
    }

    missionInterpreterArea(layerInfo, missionLayer, selectedUavList, validation, info) {

        let uavList = Object.keys(layerInfo.drawManager.options.uavList);

        if (uavList.length > 2 && uavList.includes('auto')) {
            validation = false;
            info.push(`More than one UAVs are selected for layer ${layerInfo.layer.options.name} and auto option is selected`);
            return;
        }

        // If auto is selected
        if (uavList.length == 1 && uavList[0] == 'auto') {
            for (let j = 0; j < selectedUavList.length; j++) {
                missionLayer['uavList'].push(selectedUavList[j]);
            }
        } else {
            // If auto is not selected
            for (let j = 0; j < uavList.length; j++) {
                if (selectedUavList.includes(uavList[j])) {
                    missionLayer['uavList'].push(uavList[j]);
                } else {
                    validation = false;
                    info.push(`${layerInfo.drawManager.options.name} layer has error with UAV selected. UAV associated to this layer is not selected`);
                    return;
                }
            }
        }

        missionLayer['algorithm'] = layerInfo.drawManager.options.algorithm;
        missionLayer['streetSpacing'] = layerInfo.drawManager.options.streetSpacing;
        missionLayer['wpSpace'] = layerInfo.drawManager.options.wpSpace;
    }



    missionInterpreter() {

        let selectedUavList = [];
        for (let key in this.selectedUavs) {
            if (this.selectedUavs[key]) {
                selectedUavList.push(key);
            }
        }

        if (selectedUavList.length == 0) {
            console.log("No UAV selected");
            info.push("No UAV selected");
            return [false, info, [], []];
        }

        let selectedUavListTakeOff = Object.assign([], selectedUavList);

        let validation = true;
        let info = [];
        let mission = [];
        let uavMissionList = [];
        let uavListUnique = [];

        let layersId = M.DRAW_LAYERS.getList();
        for (let i = 0; i < layersId.length; i++) {
            let layerInfo = M.DRAW_LAYERS.getDictById(layersId[i]);

            let drawManagerInfo = layerInfo.drawManager.options;
            let layer = layerInfo.layer;

            let missionLayer = {
                'name': drawManagerInfo.name,
                'height': drawManagerInfo.height,
                'uavList': [],
            }

            // Layer info
            switch (drawManagerInfo.name) {
                case 'TakeOffPoint':
                    this.missionInterpreterTakeOffPoint(layerInfo, missionLayer, selectedUavListTakeOff, validation, info);
                    break;
                case 'WayPoint':
                case 'LandPoint':
                case 'Path':
                    this.missionInterpreterMarkers(layerInfo, missionLayer, selectedUavList, validation, info);
                    break;
                case 'Area':
                    this.missionInterpreterArea(layerInfo, missionLayer, selectedUavList, validation, info);
                    break;
                default:
                    console.log('Unknown drawManager name');
                    console.log(drawManagerInfo.name);
                    break;
            }


            // Layer coordinates values
            switch (drawManagerInfo.type) {
                case 'Marker':
                    missionLayer['values'] = layer._latlng;
                    break;
                case 'Circle':
                case 'CircleMarker':
                    missionLayer['values'] = [layer._latlng, layer._mRadius];
                    break;
                case 'Line':
                case 'Polygon':
                case 'Rectangle':
                    missionLayer['values'] = layer._latlngs;
                    break;
            }

            // Add mission to all mission list. If validation is false, stop mission creation
            mission.push(missionLayer);
            if (!validation) {
                break;
            }

            // Add to uavMissionList UAV associated to this layer
            for (let j = 0; j < missionLayer['uavList'].length; j++) {
                if (missionLayer['uavList'][j] == 'auto') {
                    for (let k = 0; k < selectedUavList.length; k++) {
                        if (!uavMissionList.includes(selectedUavList[k])) {
                            uavMissionList.push(selectedUavList[k]);
                        }
                    }
                } else {
                    uavMissionList.push(missionLayer['uavList'][j]);
                }
            }
        }

        // Remove duplicates UAV id
        uavListUnique = uavMissionList.filter((v, i, a) => a.indexOf(v) === i);

        // Check if every UAV in uavMissionList has a take off point and land point in the mission
        for (let i = 0; i < uavListUnique.length; i++) {
            let uav = uavListUnique[i];
            if (uav == 'auto') {
                continue;
            }

            let takeOffFound = false;
            let landFound = false;
            for (let j = 0; j < mission.length; j++) {
                if (mission[j]['uavList'].includes(uav)) {
                    if (mission[j]['name'] == 'TakeOffPoint') {
                        takeOffFound = true;
                    } else if (mission[j]['name'] == 'LandPoint') {
                        landFound = true;
                    }
                }
            }
            if (!takeOffFound) {
                validation = false;
                info.push(`UAV ${uav} has no take off point in the mission`);
            } else if (!landFound) {
                validation = false;
                info.push(`UAV ${uav} has no land point in the mission`);
            }
        }

        // Return validation, error info, uav list associated to this mission and the mission
        if (validation) {
            return [true, info, uavListUnique, mission];
        } else {
            return [false, info, [], []];
        }
    }

    confirmBtnCallback(args = []) {
        let output = this.missionInterpreter();

        let validation = output[0];
        let info = output[1];
        let uavList = output[2];
        let mission = output[3];

        if (validation && uavList.length > 0) {
            console.log('mission is valid');
            console.log(uavList);
            console.log(mission);
            M.WS.sendRequestMissionConfirm(
                this.selectedMission,
                uavList,
                mission
            );
        } else {
            if (info.length > 0) {
                console.log("Mission validation failed");
                console.log(info);
                alert(info.join('\n'));
            } else {
                console.log("Mission validation failed");
                console.log(uavList);
                alert("Uav list is empty");
            }
        }
    }

    // #region Callbacks
    addBntDropDownMissionCallback() {
        Utils.addButtonsCallback(`${this.htmlId}-MissionList-item`, this.clickMissionListCallback.bind(this));
    }

    updateUavListCallback(myargs, args) {
        this._checkInitalize();
        this.selectedUavs[args[0]] = false;
    }

    updateMissionListCallback(myargs, args) {
        if (this.initialized) {
            HTMLUtils.updateDropDown(`${this.htmlId}-MissionList`, ['New Mission'].concat(M.MISSION_MANAGER.getList()));
            this.addBntDropDownMissionCallback();
        }
    }

    clickMissionListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-MissionList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.selectedMission = e.innerHTML;
    }

    clickUavListCallback(uavId, value) {
        this.selectedUavs[uavId] = value;

        // If there is no UAV selected, disable the confirm button
        let confirmBtn = document.getElementById(`${this.htmlId}-confirm`);
        confirmBtn.disabled = true;
        for (let key in this.selectedUavs) {
            if (this.selectedUavs[key]) {
                document.getElementById(`${this.htmlId}-confirm`).disabled = false;
                return;
            }
        }
    }

    saveMissionCallback(args) {

        let drawLayers = M.DRAW_LAYERS.getList();

        let saveInfo = {}
        for (let i = 0; i < drawLayers.length; i++) {
            let id = drawLayers[i];
            let layer_info = M.DRAW_LAYERS.getDictById(drawLayers[i]);

            let drawManager = layer_info.drawManager;
            let layer = layer_info.layer;

            saveInfo[id] = {
                'options': drawManager.options,
                'values': []
            };

            if (layer._latlng) {
                saveInfo[id]['values'] = layer._latlng;
            } else if (layer._latlngs) {
                saveInfo[id]['values'] = layer._latlngs;
            }

        }

        if (Object.keys(saveInfo).length > 0) {
            console.log("Saving");
            console.log(saveInfo);
            Utils.download(`DrawMission.txt`, saveInfo);
        }
    }

    loadMissionCallback(args, input) {
        let fileToLoad = input.target.files[0];
        Utils.loadFile(fileToLoad, this.loadMissionFileCallback.bind(this), 'json', fileToLoad.name.split('.')[0]);
    }

    loadMissionFileCallback(data, myargs) {

        let input = document.getElementById(`${this.htmlId}-missionFile`);
        input.value = '';

        for (let i = 0; i < Object.keys(data).length; i++) {
            let id = Object.keys(data)[i];
            let info = data[id];

            let options = info.options;
            let values = info.values;

            switch (options.type) {
                case 'Marker':
                    switch (options.name) {
                        case 'TakeOffPoint':
                            console.log("TakeOffPoint");
                            console.log(values);
                            console.log(options)
                            this.takeOffPoint.codeDraw(values, options);
                            break;
                        case 'LandPoint':
                            this.landPoint.codeDraw(values, options);
                            break;
                        case 'WayPoint':
                            this.wayPoint.codeDraw(values, options);
                            break;
                        default:
                            throw new Error(`Unknown marker type ${options.name}`);
                            break;
                    }
                    break;
                case 'Line':
                    switch (options.name) {
                        case 'Path':
                            this.path.codeDraw(values, options);
                            break;
                        default:
                            throw new Error(`Unknown line type ${options.name}`);
                    }
                    break;
                case 'Polygon':
                    switch (options.name) {
                        case 'Area':
                            this.area.codeDraw(values, options);
                            break;
                        default:
                            throw new Error(`Unknown polygon type ${options.name}`);
                    }
                    break;
                default:
                    throw new Error(`Unknown type ${options.name}`);
                    break;
            }
        }

        Utils.resetLoadFileInput(`${this.htmlId}-missionFile`);
    }

    // #endregion

    // #endregion
}
