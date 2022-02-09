class DropDown extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'btn': args[0],
                'expand': args[1],
            }
        );
    }  

    static addHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'row');
        
        HTMLUtils.addHTML(div, content.btn);

        let UL = document.createElement('ul');
        UL.setAttribute('class', 'dropdown-menu');
        UL.setAttribute('id', content.id + '-menu');

        HTMLUtils.addHTML(UL, content.expand);
        div.appendChild(UL);
        
        return div;
    }
}

class DropDownBtn extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'text': args[0],
            }
        );
    }  

    static addHTML(content) {


        let button = document.createElement('button');
        button.setAttribute('type', 'button');
        button.innerHTML = content.text;

        button.setAttribute('class', 'dropdown-toggle');
        button.setAttribute('data-bs-toggle', 'dropdown');
        button.setAttribute('aria-expanded', 'false');

        return button;
    }
}

class DropDownExpand extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
            }
        );
    }  

    static addHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'row');
        
        for (let i = 0; i < content.list.length; i++) {
            let Li = document.createElement('li');
            content.list[i].attributes['class'] += 'dropdown-item';
            HTMLUtils.addHTML(Li, content.list[i]);
            div.appendChild(Li);
        }

        return div;
    }
}