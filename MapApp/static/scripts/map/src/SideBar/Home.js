class Home
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let homeHtmlList = [];

        // Go to
        let goToLat  = HTMLUtils.addDict('input', 'sideBar-left-home-content-goToLat', {'class': 'form-control', 'required': 'required',}, 'text', 'Latitude');
        let goToLon  = HTMLUtils.addDict('input', 'sideBar-left-home-content-goToLon', {'class': 'form-control', 'required': 'required',}, 'text', 'Longitude');
        let goToBtn  = HTMLUtils.addDict('button', 'sideBar-left-home-content-goToBtn', {'class': 'btn btn-primary'}, 'Go to');
        let goToRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [goToLat, goToLon, goToBtn], {'class': 'col-md-4'});

        let gotoCollapse = HTMLUtils.addDict('collapse', 'sideBar-left-home-content-gotoCollapse', {}, 'Go to', false, [goToRow]);

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
