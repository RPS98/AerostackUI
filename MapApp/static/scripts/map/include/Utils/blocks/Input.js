class Input extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'inputType': args[0],
                'placeholder': args[1],
            }
        );
    }   

    static addHTML(content) {
        let input = document.createElement('input');
        input.setAttribute('type', content.inputType);
        input.setAttribute('placeholder', content.placeholder);
        return input;
    }
}