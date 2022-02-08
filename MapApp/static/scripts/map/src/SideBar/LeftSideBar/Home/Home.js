class Home
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let homeHtmlList = [];

        // Go to
        let latLonAttributes = {
            'class': 'form-control',
            'required': 'required',
        };
        let latLonBtnAttributes = {
            'class': 'btn btn-primary',
        }

        let goToLat = HTMLUtils.addDict('input', 'sideBar-left-home-content-goToLat', latLonAttributes, 'text', 'Latitude');
        let goToLon = HTMLUtils.addDict('input', 'sideBar-left-home-content-goToLon', latLonAttributes, 'text', 'Longitude');
        let goToBtn = HTMLUtils.addDict('button', 'sideBar-left-home-content-goToBtn', latLonBtnAttributes, 'Go to');
        let goToRow = HTMLUtils.addDict('row', 'none', {}, [goToLat, goToLon, goToBtn]);
        let gotoCollapse = HTMLUtils.addDict('collapse', 'sideBar-left-home-content-gotoCollapse', {}, 'Go to', [goToRow]);


        homeHtmlList.push(gotoCollapse);

        HTMLUtils.addToExistingElement('sideBar-left-home-content', homeHtmlList);
    }

    addCallbacks() {
        Utils.addFormCallback(
            'sideBar-left-home-content-goToBtn', 
            ['sideBar-left-home-content-goToLat', 'sideBar-left-home-content-goToLon'], 
            ['map_center_lat', 'map_center_long'], 
            Home.goToCallback,
            19 // zoom
        );
    }

    static goToCallback(zoom, input) {
        console.log('goToCallback');
        console.log(map)
        map.flyTo([input['map_center_lat'], input['map_center_long']], zoom);
    }
}