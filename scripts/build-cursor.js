const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

const packageJsonPath = path.join(__dirname, '..', 'package.json');

function readPackageJson() {
    const content = fs.readFileSync(packageJsonPath, 'utf8');
    return JSON.parse(content);
}

function writePackageJson(packageJson) {
    fs.writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 4) + '\n');
}

function backupPackageJson() {
    const backupPath = packageJsonPath + '.backup';
    fs.copyFileSync(packageJsonPath, backupPath);
    console.log('Backed up package.json to package.json.backup');
}

function restorePackageJson() {
    const backupPath = packageJsonPath + '.backup';
    if (fs.existsSync(backupPath)) {
        fs.copyFileSync(backupPath, packageJsonPath);
        fs.unlinkSync(backupPath);
        console.log('Restored package.json from backup');
    }
}

function modifyForCursor(packageJson) {
    // Change ms-vscode.cpptools to anysphere.cpptools
    if (packageJson.extensionDependencies) {
        packageJson.extensionDependencies = packageJson.extensionDependencies.map(dep => {
            if (dep === 'ms-vscode.cpptools') {
                return 'anysphere.cpptools';
            }
            return dep;
        });
    }
    return packageJson;
}

function buildExtension() {
    console.log('Building extension for Cursor...');
    execSync('npm run package', { stdio: 'inherit' });
    console.log('Extension built successfully for Cursor!');
}

function main() {
    try {
        console.log('Starting Cursor build process...');
        
        // Backup original package.json
        backupPackageJson();
        
        // Read and modify package.json
        const packageJson = readPackageJson();
        const modifiedPackageJson = modifyForCursor(packageJson);
        
        // Write modified package.json
        writePackageJson(modifiedPackageJson);
        console.log('Modified package.json for Cursor compatibility');
        
        // Build the extension
        buildExtension();
        
        // Restore original package.json
        restorePackageJson();
        
        console.log('Cursor build process completed successfully!');
        
    } catch (error) {
        console.error('Error during Cursor build process:', error);
        
        // Always try to restore package.json on error
        try {
            restorePackageJson();
        } catch (restoreError) {
            console.error('Failed to restore package.json:', restoreError);
        }
        
        process.exit(1);
    }
}

main(); 