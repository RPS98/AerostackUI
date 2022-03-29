class Home {
    constructor() {
        this.htmlId = 'sideBar-left-home-content';

        this.defaultGoTo = [28.14376, -16.50235];

        this.defaultUAVname = 'UAV_name';
        this.defaultUAV = [28.1439717, -16.5032634];

        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let homeHtmlList = [];

        // Go to
        let goToLat = HTMLUtils.addDict('input', `${this.htmlId}-goToLat`, { 'class': 'form-control', 'required': 'required', 'value': this.defaultGoTo[0] }, 'number', 'Latitude');
        let goToLng = HTMLUtils.addDict('input', `${this.htmlId}-goToLng`, { 'class': 'form-control', 'required': 'required', 'value': this.defaultGoTo[1] }, 'number', 'Longitude');
        let goToBtn = HTMLUtils.addDict('button', `${this.htmlId}-goToBtn`, { 'class': 'btn btn-primary' }, 'Go to');
        let goToRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [goToLat, goToLng, goToBtn], { 'class': 'col-md-4' });
        let gotoCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-gotoCollapse`, {}, 'Go to', true, [goToRow]);

        // Add virtual UAV button
        let virtualUAVName = HTMLUtils.addDict('input', `${this.htmlId}-virtualName`, { 'class': 'form-control', 'required': 'required', 'value': this.defaultUAVname}, 'text', 'Name');
        let virtualUAVLat = HTMLUtils.addDict('input', `${this.htmlId}-virtualLat`, { 'class': 'form-control', 'required': 'required', 'value': this.defaultUAV[0] }, 'number', 'Latitude');
        let virtualUAVLon = HTMLUtils.addDict('input', `${this.htmlId}-virtualLng`, { 'class': 'form-control', 'required': 'required', 'value': this.defaultUAV[1] }, 'number', 'Longitude');
        let virtualUAVPose = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [virtualUAVLat, virtualUAVLon], { 'class': 'col-md-6' });

        let virtualBtnContent = HTMLUtils.addDict('button', `${this.htmlId}-virtualBtn`, { 'class': 'btn btn-primary' }, 'Add virtual UAV');
        let virtualBtn = HTMLUtils.addDict('splitDivs', 'none', {}, [virtualBtnContent], { 'class': 'row m-1' });

        let virtualCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-virtualCollapse`, {}, 'Add virtual UAV', true, [virtualUAVName, virtualUAVPose, virtualBtn]);


        homeHtmlList.push(gotoCollapse);
        homeHtmlList.push(virtualCollapse);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, homeHtmlList);
    }

    addCallbacks() {
        Utils.addFormCallback(
            `${this.htmlId}-goToBtn`,
            [`${this.htmlId}-goToLat`, `${this.htmlId}-goToLng`],
            ['map_center_lat', 'map_center_lng'],
            Home.goToCallback,
            19 // zoom
        );

        Utils.addFormCallback(
            `${this.htmlId}-virtualBtn`,
            [`${this.htmlId}-virtualName`, `${this.htmlId}-virtualLat`, `${this.htmlId}-virtualLng`],
            ['name', 'lat', 'lng'],
            this.addUAVCallback.bind(this)
        );
    }

    static goToCallback(zoom, input) {
        console.log('goToCallback');
        M.MAP.flyTo([input['map_center_lat'], input['map_center_lng']], zoom);
    }

    addUAVCallback(arg, input) {

        let uavInfo = {
            'id': input.name,
            'state': {},
            'pose': {'lat': parseFloat(input.lat), 'lng': parseFloat(input.lng), 'height': 0, 'yaw': 0}
        }

        M.WS.sendUavInfo(uavInfo);
    }
}
