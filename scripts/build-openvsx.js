const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

function backupPackageJson() {
    const backupPath = 'package.json.backup';
    if (fs.existsSync(backupPath)) {
        fs.unlinkSync(backupPath);
    }
    fs.copyFileSync('package.json', backupPath);
    console.log('Backed up package.json to package.json.backup');
}

function readPackageJson() {
    const packageJsonPath = path.join(__dirname, '..', 'package.json');
    const packageJsonContent = fs.readFileSync(packageJsonPath, 'utf8');
    return JSON.parse(packageJsonContent);
}

function writePackageJson(packageJson) {
    const packageJsonPath = path.join(__dirname, '..', 'package.json');
    fs.writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 2));
}

function restorePackageJson() {
    const backupPath = 'package.json.backup';
    if (fs.existsSync(backupPath)) {
        fs.copyFileSync(backupPath, 'package.json');
        fs.unlinkSync(backupPath);
        console.log('Restored original package.json');
    }
}

function modifyForOpenVSX(packageJson) {
    // Update VS Code engine version from ^1.101.0 to ^1.99.0
    if (packageJson.engines && packageJson.engines.vscode) {
        packageJson.engines.vscode = packageJson.engines.vscode.replace('^1.101.0', '^1.99.0');
        console.log('Updated VS Code engine version to ^1.99.0');
    }
    
    // Update @types/vscode version from ^1.101.0 to ^1.99.0
    if (packageJson.devDependencies && packageJson.devDependencies['@types/vscode']) {
        packageJson.devDependencies['@types/vscode'] = packageJson.devDependencies['@types/vscode'].replace('^1.101.0', '^1.99.0');
        console.log('Updated @types/vscode version to ^1.99.0');
    }
    if (packageJson.extensionDependencies) {
        packageJson.extensionDependencies = packageJson.extensionDependencies.map(dep => {
            if (dep === 'ms-vscode.cpptools') {
                return 'vadimcn.vscode-lldb';
            }
            return dep;
        });
    }  
    
    return packageJson;
}

function buildExtension() {
    console.log('Building extension for OpenVSX...');
    execSync('npm run package', { stdio: 'inherit' });
    console.log('Extension built successfully for OpenVSX!');
}

function main() {
    try {
        console.log('Starting OpenVSX build process...');
        
        // Backup original package.json
        backupPackageJson();
        
        // Read and modify package.json
        const packageJson = readPackageJson();
        const modifiedPackageJson = modifyForOpenVSX(packageJson);
        
        // Write modified package.json
        writePackageJson(modifiedPackageJson);
        console.log('Modified package.json for OpenVSX compatibility');
        
        // Build the extension
        buildExtension();
        
        // Restore original package.json
        //restorePackageJson();
        
        console.log('OpenVSX build process completed successfully!');
        
    } catch (error) {
        console.error('Error during OpenVSX build process:', error);
        
        // Restore original package.json on error
        restorePackageJson();
        
        process.exit(1);
    }
}

main(); 
