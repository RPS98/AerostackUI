class Home
{
    constructor() {
        this.htmlId = 'sideBar-left-home-content';

        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let homeHtmlList = [];

        // Go to
        let goToLat  = HTMLUtils.addDict('input',  `${this.htmlId}-goToLat`, {'class': 'form-control', 'required': 'required',}, 'text', 'Latitude');
        let goToLon  = HTMLUtils.addDict('input',  `${this.htmlId}-goToLon`, {'class': 'form-control', 'required': 'required',}, 'text', 'Longitude');
        let goToBtn  = HTMLUtils.addDict('button', `${this.htmlId}-goToBtn`, {'class': 'btn btn-primary'}, 'Go to');
        let goToRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [goToLat, goToLon, goToBtn], {'class': 'col-md-4'});

        let gotoCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-gotoCollapse`, {}, 'Go to', false, [goToRow]);

        homeHtmlList.push(gotoCollapse);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, homeHtmlList);
    }

    addCallbacks() {
        Utils.addFormCallback(
            `${this.htmlId}-goToBtn`, 
            [`${this.htmlId}-goToLat`, `${this.htmlId}-goToLon`], 
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
